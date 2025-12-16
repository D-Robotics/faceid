// Copyright (c) 2025，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FEATURE_MANAGE_H_
#define FEATURE_MANAGE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "dnn_node/dnn_node_data.h"

#include "include/database.h"
#include "include/item.h"

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::NV12PyramidInput;

class TrackIdResult {
 public:  
  std::vector<int> ids;

  void Reset() {ids.clear();}
};

class FeatureManage {
 public:
  FeatureManage(std::string db_file, int feature_size, float threshold);
  ~FeatureManage() {}

  // 对于roi infer task，每个roi对应一次Parse
  // 因此需要在Parse中实现output和roi的match处理，即当前的Parse对应的是那个roi
  int32_t Parse(
      std::shared_ptr<TrackIdResult> &output,
      std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
      std::shared_ptr<std::vector<hbDNNRoi>> rois,
      std::shared_ptr<NV12PyramidInput> pyramid = nullptr);

 private:
  int UpdateReid(
      std::vector<int32_t> &feature,
      const int roi_idx,
      std::shared_ptr<TrackIdResult> &output);

  int Query(const int32_t *data,
            std::vector<Item>& target_items);

  int Render(const std::shared_ptr<NV12PyramidInput>& pyramid,
              hbDNNRoi &roi,
              std::string &file_name);

  ItemDatabase db;
  int feature_size_ = 128;
  float threshold_ = 0.75;
};

#endif  // FEATURE_MANAGE_H_
