/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <basalt/calibration/cam_calib.h>

#include <basalt/optimization/poses_optimize.h>

#include <basalt/serialization/headers_serialization.h>

#include <iostream>
#include <opencv2/highgui.hpp>

#include "basalt/utils/common_types.h"

namespace basalt {

CamCalib::CamCalib(const std::string& aprilgrid_path,
                   const std::vector<std::string>& cam_types,
                   double huber_thresh, double stop_thresh, bool load_corners)
    : april_grid(aprilgrid_path),
      cam_types(cam_types),
      huber_thresh(huber_thresh),
      stop_thresh(stop_thresh),
      load_corners(load_corners) {}

CamCalib::~CamCalib() {}

const AprilGrid& CamCalib::GetAprilGrid() { return april_grid; }
void CamCalib::setImageSize(size_t width, size_t height) {
  this->image_width = width;
  this->image_height = height;

  corners_distribution = cv::Mat(cv::Size(image_width, image_height), CV_8UC3,
                                 cv::Scalar(255, 255, 255));
}

void CamCalib::optimizeUntilConvergence() {
  bool converged = false;

  while (!converged) {
    converged = optimizeWithParam(true);
  }
}

void CamCalib::setCornerDetectionImage(const cv::Mat& corners_image) {
  corners_detection = corners_image;
}

bool CamCalib::detectCorners(const ManagedImage<uint16_t>::Ptr& image,
                             FrameId& frame_count, std::string& path) {
  if (load_corners) {
    std::ifstream is(path, std::ios::binary);

    if (is.good()) {
      cereal::BinaryInputArchive archive(is);

      calib_corners.clear();
      calib_corners_rejected.clear();
      archive(calib_corners);
      archive(calib_corners_rejected);

      frame_count = static_cast<FrameId>(calib_corners.size());

      std::cout << "Loaded detected corners from: " << path << std::endl;
    } else {
      std::cout << "No pre-processed detected corners found" << std::endl;
    }
    load_corners = false;
  }
  std ::cout << "Started detecting corners" << std::endl;

  size_t num_corners_before = this->calib_corners.size();
  CalibHelper::detectCorners(image, this->april_grid, this->calib_corners,
                             this->calib_corners_rejected, frame_count);
  for (auto& corner : this->calib_corners) {
    for (auto& point : corner.second.corners) {
      cv::circle(corners_distribution, cv::Point2f(point.x(), point.y()), 1,
                 cv::Scalar(0, 255, 0), 3);
    }
  }

  size_t num_corners_after = this->calib_corners.size();

  bool found = num_corners_after > num_corners_before;
  if (found) {
    TimeCamId tcid(static_cast<FrameId>(num_corners_after - 1), 0);
    auto& corner = this->calib_corners.at(tcid);
    for (auto& point : corner.corners) {
      cv::circle(corners_detection, cv::Point2f(point.x(), point.y()), 1,
                 cv::Scalar(0, 255, 0), 3);
    }
  }

  return found;
}

void CamCalib::getCornersDistribution(cv::Mat& image) {
  image = corners_distribution;
}

void CamCalib::getCornersDetection(cv::Mat& image) {
  image = corners_detection;
}

const CalibCornerMap& CamCalib::getCorners() { return this->calib_corners; }

void CamCalib::saveCorners(const std::string& path) {
  std::ofstream os(path, std::ios::binary);
  cereal::BinaryOutputArchive archive(os);

  archive(this->calib_corners);
  archive(this->calib_corners_rejected);

  std::cout << "Done detecting corners. Saved them here: " << path << std::endl;
}

void CamCalib::initCamIntrinsics() {
  if (calib_corners.empty()) {
    std::cerr << "No corners detected. Press detect_corners to start corner "
                 "detection."
              << std::endl;
    return;
  }

  std::cout << "Started camera intrinsics initialization" << std::endl;

  if (!calib_opt) {
    calib_opt.reset(new PosesOptimization);
  }

  calib_opt->resetCalib(1, cam_types);

  std::vector<bool> cam_initialized(1, false);

  int inc = 1;
  size_t num_frames = this->calib_corners.size();

  for (size_t i = 0; i < num_frames; i += inc) {
    TimeCamId tcid(static_cast<FrameId>(i), 0);

    if (calib_corners.find(tcid) != calib_corners.end()) {
      CalibCornerData cid = calib_corners.at(tcid);

      Eigen::Vector4d init_intr;

      bool success = CalibHelper::initializeIntrinsics(
          cid.corners, cid.corner_ids, april_grid, this->image_width,
          this->image_height, init_intr);

      if (success) {
        cam_initialized[0] = true;
        calib_opt->calib->intrinsics[0].setFromInit(init_intr);
        break;
      }
    }
  }
  if (!cam_initialized[0]) {
    std::vector<CalibCornerData*> pinhole_corners;
    int w = 0;
    int h = 0;

    for (size_t i = 0; i < num_frames; i += inc) {
      TimeCamId tcid(static_cast<FrameId>(i), 0);

      auto it = calib_corners.find(tcid);
      if (it != calib_corners.end()) {
        if (it->second.corners.size() > 8) {
          pinhole_corners.emplace_back(&it->second);
        }
      }

      std::cout << pinhole_corners.size() << std::endl;

      w = image_width;
      h = image_height;
    }

    BASALT_ASSERT(w > 0 && h > 0);

    Eigen::Vector4d init_intr;

    bool success = CalibHelper::initializeIntrinsicsPinhole(
        pinhole_corners, april_grid, w, h, init_intr);

    if (success) {
      cam_initialized[0] = true;

      std::cout << "Initialized camera " << 0
                << " with pinhole model. You should set pinhole model for "
                   "this camera!"
                << std::endl;
      calib_opt->calib->intrinsics[0].setFromInit(init_intr);
    }
  }

  std::cout << "Done camera intrinsics initialization:" << std::endl;
  std::cout << "Cam " << 0 << ": "
            << calib_opt->calib->intrinsics[0].getParam().transpose()
            << std::endl;

  // set resolution
  {
    Eigen::aligned_vector<Eigen::Vector2i> res;
    res.emplace_back(this->image_width, this->image_height);

    calib_opt->setResolution(res);
  }
}

void CamCalib::initCamPoses() {
  if (calib_corners.empty()) {
    std::cerr << "No corners detected. Press detect_corners to start corner "
                 "detection."
              << std::endl;
    return;
  }

  if (!calib_opt.get() || !calib_opt->calibInitialized()) {
    std::cerr << "No initial intrinsics. Press init_intrinsics initialize "
                 "intrinsics"
              << std::endl;
    return;
  }

  std::cout << "Started initial camera pose computation " << std::endl;

  const Calibration<double>* test = calib_opt->calib.get();

  CalibHelper::initCamPoses(calib_opt->calib.get(),
                            april_grid.aprilgrid_corner_pos_3d,
                            this->calib_corners, this->calib_init_poses);

  std::cout << "Done initial camera pose computation.  " << std::endl;
}

void CamCalib::initCamExtrinsics() {
  if (calib_init_poses.empty()) {
    std::cerr << "No initial camera poses. Press init_cam_poses initialize "
                 "camera poses "
              << std::endl;
    return;
  }

  if (!calib_opt.get() || !calib_opt->calibInitialized()) {
    std::cerr << "No initial intrinsics. Press init_intrinsics initialize "
                 "intrinsics"
              << std::endl;
    return;
  }

  std::map<std::pair<size_t, size_t>, std::pair<int, int64_t>> cam_graph;

  // Construct the graph.

  size_t num_frames = this->calib_corners.size();
  for (size_t i = 0; i < num_frames; i++) {
    TimeCamId tcid_i(static_cast<FrameId>(i), 0);

    auto it = calib_init_poses.find(tcid_i);
    if (it == calib_init_poses.end() || it->second.num_inliers < MIN_CORNERS)
      continue;
  }

  std::vector<bool> cameras_initialized(1, false);
  cameras_initialized[0] = true;
  size_t last_camera = 0;
  calib_opt->calib->T_i_c[0] = Sophus::SE3d();  // Identity

  auto next_max_weight_edge = [&](size_t cam_id) {
    int max_weight = -1;
    std::pair<int, int64_t> res(-1, -1);

    for (size_t i = 0; i < 1; i++) {
      if (cameras_initialized[i]) continue;

      std::pair<size_t, size_t> edge_id;

      if (i < cam_id) {
        edge_id = std::make_pair(i, cam_id);
      } else if (i > cam_id) {
        edge_id = std::make_pair(cam_id, i);
      }

      auto it = cam_graph.find(edge_id);
      if (it != cam_graph.end() && max_weight < it->second.first) {
        max_weight = it->second.first;
        res.first = i;
        res.second = it->second.second;
      }
    }

    return res;
  };

  std::cout << "Done camera extrinsics initialization:" << std::endl;
  for (size_t j = 0; j < 1; j++) {
    std::cout << "T_c0_c" << j << ":\n"
              << calib_opt->calib->T_i_c[j].matrix() << std::endl;
  }
}  // namespace basalt

void CamCalib::initOptimization() {
  if (!calib_opt) {
    std::cerr << "Calibration is not initialized. Initialize calibration first!"
              << std::endl;
    return;
  }

  if (calib_init_poses.empty()) {
    std::cerr << "No initial camera poses. Press init_cam_poses initialize "
                 "camera poses "
              << std::endl;
    return;
  }

  calib_opt->setAprilgridCorners3d(april_grid.aprilgrid_corner_pos_3d);

  std::unordered_set<TimeCamId> invalid_frames;
  for (const auto& kv : calib_corners) {
    if (kv.second.corner_ids.size() < MIN_CORNERS)
      invalid_frames.insert(kv.first);
  }

  size_t num_frames = this->calib_corners.size();
  for (size_t j = 0; j < num_frames; ++j) {
    int max_inliers = -1;
    int max_inliers_idx = -1;

    for (size_t cam_id = 0; cam_id < calib_opt->calib->T_i_c.size(); cam_id++) {
      TimeCamId tcid(static_cast<FrameId>(j), cam_id);
      const auto cp_it = calib_init_poses.find(tcid);
      if (cp_it != calib_init_poses.end()) {
        if ((int)cp_it->second.num_inliers > max_inliers) {
          max_inliers = cp_it->second.num_inliers;
          max_inliers_idx = cam_id;
        }
      }
    }

    if (max_inliers >= (int)MIN_CORNERS) {
      TimeCamId tcid(static_cast<FrameId>(j), max_inliers_idx);
      const auto cp_it = calib_init_poses.find(tcid);

      // Initial pose
      calib_opt->addPoseMeasurement(
          static_cast<FrameId>(j),
          cp_it->second.T_a_c *
              calib_opt->calib->T_i_c[max_inliers_idx].inverse());
    } else {
      // Set all frames invalid if we do not have initial pose
      for (size_t cam_id = 0; cam_id < calib_opt->calib->T_i_c.size();
           cam_id++) {
        invalid_frames.emplace(static_cast<FrameId>(j), cam_id);
      }
    }
  }

  for (const auto& kv : calib_corners) {
    if (invalid_frames.count(kv.first) == 0)
      calib_opt->addAprilgridMeasurement(kv.first.frame_id, kv.first.cam_id,
                                         kv.second.corners,
                                         kv.second.corner_ids);
  }

  calib_opt->init();

  std::cout << "Initialized optimization." << std::endl;
}  // namespace basalt

void CamCalib::optimize() { optimizeWithParam(true); }

bool CamCalib::optimizeWithParam(bool print_info,
                                 std::map<std::string, double>* stats) {
  if (calib_init_poses.empty()) {
    std::cerr << "No initial camera poses. Press init_cam_poses initialize "
                 "camera poses "
              << std::endl;
    return true;
  }

  if (!calib_opt.get() || !calib_opt->calibInitialized()) {
    std::cerr << "No initial intrinsics. Press init_intrinsics initialize "
                 "intrinsics"
              << std::endl;
    return true;
  }

  bool converged = true;

  if (calib_opt) {
    double error;
    double reprojection_error;
    int num_points;

    auto start = std::chrono::high_resolution_clock::now();

    converged = calib_opt->optimize(true, huber_thresh, stop_thresh, error,
                                    num_points, reprojection_error);

    auto finish = std::chrono::high_resolution_clock::now();

    if (stats) {
      stats->clear();

      stats->emplace("energy_error", error);
      stats->emplace("num_points", num_points);
      stats->emplace("mean_energy_error", error / num_points);
      stats->emplace("reprojection_error", reprojection_error);
      stats->emplace("mean_reprojection_error",
                     reprojection_error / num_points);
    }

    if (print_info) {
      std::cout << "==================================" << std::endl;

      std::cout << "intrinsics " << 0 << ": "
                << calib_opt->calib->intrinsics[0].getParam().transpose()
                << std::endl;
      std::cout << "T_i_c" << 0 << ":\n"
                << calib_opt->calib->T_i_c[0].matrix() << std::endl;

      std::cout << "Current error: " << error << " num_points " << num_points
                << " mean_error " << error / num_points
                << " reprojection_error " << reprojection_error
                << " mean reprojection " << reprojection_error / num_points
                << " opt_time "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       finish - start)
                       .count()
                << "ms." << std::endl;

      if (converged) std::cout << "Optimization Converged !!" << std::endl;

      std::cout << "==================================" << std::endl;
    }
  }

  return converged;
}

void CamCalib::saveCalib(double& fx, double& fy, double& cx, double& cy,
                         double& k0, double& k1, double& k2, double& k3) {
  if (calib_opt) {
    Eigen::VectorXd values =
        calib_opt->calib->intrinsics[0].getParam().transpose();

    fx = values[0];
    fy = values[1];
    cx = values[2];
    cy = values[3];

    k0 = values[4];
    k1 = values[5];
    k2 = values[6];
    k3 = values[7];

    std::cout << " fx: " << fx << " fy: " << fy << " cx: " << cx << " cy: " << cy
              << " k0: " << k0 << " k1: " << k1 << " k2: " << k2 << " k3: " << k3
              << std::endl;

    std::cout << "Saved calibration " << std::endl;
  }
}

}  // namespace basalt
