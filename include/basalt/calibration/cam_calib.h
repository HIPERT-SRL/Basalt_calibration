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
#pragma once

// #include <pangolin/display/image_view.h>
// #include <pangolin/gl/gldraw.h>
// #include <pangolin/image/image.h>
// #include <pangolin/image/image_io.h>
// #include <pangolin/image/typed_image.h>
// #include <pangolin/pangolin.h>

#include "Eigen/Dense"

#include <iostream>
#include <limits>
#include <thread>

#include <basalt/calibration/aprilgrid.h>
#include <basalt/calibration/calibration_helper.h>
#include <basalt/image/image.h>
#include <basalt/io/dataset_io.h>
#include <basalt/io/dataset_io_custom.h>
#include <basalt/utils/sophus_utils.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace basalt {

class PosesOptimization;

class CamCalib {
 public:
  CamCalib(const std::string& aprilgrid_path,
           const std::vector<std::string>& cam_types, double huber_thresh,
           double stop_thresh, bool load_corners, int numCams);

  ~CamCalib();

  bool detectCorners(const ManagedImage<uint16_t>::Ptr& image, int camId,
                     FrameId& frame_count, std::string& path);

  void detectCornersMultiThread();
  void setImageSize(size_t width, size_t height);

  void addImage(const ManagedImage<uint16_t>::Ptr& image, int camId,
                int64_t timestamp_ns);

  const CalibCornerMap& getCorners();

  std::string serializeCalib();

  void saveCorners(const std::string& path);

  void computeProjections();

  void setCornerDetectionImage(const cv::Mat& corners_image);

  void getCornersDistribution(cv::Mat& image);

  void getCornersDetection(cv::Mat& image);

  const AprilGrid& GetAprilGrid();

  void initCamIntrinsics();

  void initCamPoses();

  void initCamExtrinsics();

  void initOptimization();

  double optimizeUntilConvergence();

  void optimize();

  std::pair<bool, double>  optimizeWithParam(bool print_info,
                         std::map<std::string, double>* stats = nullptr);

  void saveCalib(int camId, double& fx, double& fy, double& cx, double& cy, double& k0,
                 double& k1, double& k2, double& k3, Eigen::Matrix4f& T_i_c);

  std::shared_ptr<CustomVioDataset> getDataset() { return dataset; }
  CalibCornerMap& getCalibCorners() { return calib_corners; }
  CalibCornerMap& getCalibCornersRejected() { return calib_corners_rejected; }
  CalibInitPoseMap& getCalibInitPoses() { return calib_init_poses; }
  std::map<TimeCamId, ProjectedCornerData>& getReprojectedCorners() {
    return reprojected_corners;
  }

 private:
  static constexpr int UI_WIDTH = 300;

  static constexpr size_t RANSAC_THRESHOLD = 10;

  int image_width;
  int image_height;

  std::shared_ptr<CustomVioDataset> dataset;
  CalibCornerMap calib_corners;
  CalibCornerMap calib_corners_rejected;
  CalibInitPoseMap calib_init_poses;

  std::map<TimeCamId, ProjectedCornerData> reprojected_corners;
  std::shared_ptr<PosesOptimization> calib_opt;

  AprilGrid april_grid;

  std::vector<std::string> cam_types;

  const size_t MIN_CORNERS = 15;

  double huber_thresh;
  double stop_thresh;

  bool load_corners;

  cv::Mat corners_distribution;
  cv::Mat corners_detection;
};

}  // namespace basalt
