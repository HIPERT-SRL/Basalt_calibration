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
#ifndef DATASET_IO_CUSTOM_H
#define DATASET_IO_CUSTOM_H


#include <basalt/io/dataset_io.h>
#include <optional>
#include <unordered_map>
#include <vector>


namespace basalt {

class CustomVioDataset : public VioDataset {
  size_t num_cams = 1;

  std::vector<int64_t> image_timestamps;
  std::unordered_set<int64_t> timestamp_set;


  // vector of images for every timestamp
  // assumes vectors size is num_cams for every timestamp with null pointers for
  // missing frames
  std::unordered_map<int64_t, std::vector<std::optional<ImageData>>> image_data_idx;

  Eigen::aligned_vector<AccelData> accel_data;
  Eigen::aligned_vector<GyroData> gyro_data;

  std::vector<int64_t> gt_timestamps;  // ordered gt timestamps
  Eigen::aligned_vector<Sophus::SE3d>
      gt_pose_data;  // TODO: change to eigen aligned

  int64_t mocap_to_imu_offset_ns;

 public:
  ~CustomVioDataset() {}

  size_t get_num_cams() const { return num_cams; }

  void set_num_cams(size_t num_cams) { this->num_cams = num_cams; }

void add_imu_data(int64_t timestamp_ns, double ax, double ay, double az,
                  double wx, double wy, double wz) {

    this->accel_data.emplace_back();
    this->accel_data.back().timestamp_ns = timestamp_ns;
    this->accel_data.back().data = Eigen::Vector3d(ax, ay, az);

    this->gyro_data.emplace_back();
    this->gyro_data.back().timestamp_ns = timestamp_ns;
    this->gyro_data.back().data = Eigen::Vector3d(wx, wy, wz);

    if (accel_data.size() > 1) {
        int64_t prev_ts = accel_data[accel_data.size() - 2].timestamp_ns;
        int64_t curr_ts = accel_data.back().timestamp_ns;

        if (curr_ts <= prev_ts) {
            std::cout << "IMU TIMESTAMP NOT ORDERED! " << std::endl;
        }
    }
}


  void add_image(const ManagedImage<uint8_t>::Ptr& image,
              int camId,
              int64_t timestamp_ns)
  {
      assert(camId >= 0 && camId < num_cams);
      if (image_data_idx.find(timestamp_ns) == image_data_idx.end()) {
          image_timestamps.push_back(timestamp_ns);
          image_data_idx[timestamp_ns] =
              std::vector<std::optional<ImageData>>(num_cams);
      }
      ImageData imgData;
      imgData.img = image;
      imgData.exposure = 1.0f;
      image_data_idx[timestamp_ns][camId] = imgData;
  }
  void add_timestamp(int64_t timestamp_ns)
  {
      if (timestamp_set.count(timestamp_ns) == 0) {
          timestamp_set.insert(timestamp_ns);
          image_timestamps.push_back(timestamp_ns);
      }
  }

  std::vector<int64_t> &get_image_timestamps() { return image_timestamps; }

  const Eigen::aligned_vector<AccelData> &get_accel_data() const {
    return accel_data;
  }
  const Eigen::aligned_vector<GyroData> &get_gyro_data() const {
    return gyro_data;
  }
  const std::vector<int64_t> &get_gt_timestamps() const {
    return gt_timestamps;
  }
  const Eigen::aligned_vector<Sophus::SE3d> &get_gt_pose_data() const {
    return gt_pose_data;
  }

  int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

  std::vector<ImageData> get_image_data(int64_t t_ns) {
    std::vector<ImageData> res(num_cams);

    auto it = image_data_idx.find(t_ns);

    if (it != image_data_idx.end())
      for (size_t i = 0; i < num_cams; i++) {

        if (!it->second[i].has_value()) continue;
        res[i] = it->second[i].value();
      }
    return res;
  }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace basalt

#endif
