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
#include "include/model_adapter.h"
#include "include/lsh_index.h"

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::NV12PyramidInput;

class TrackIdResult {
 public:  
  std::vector<int> ids;

  void Reset() {ids.clear();}
};

namespace faceid {
  class ModelAdapter;
  enum class ModelType;
}

class FeatureManage {
 public:
   FeatureManage(std::string db_file, int feature_size, float threshold);
   // New constructor with model type
   FeatureManage(std::string db_file, int feature_size, float threshold, 
                 faceid::ModelType model_type);
   ~FeatureManage() {}

  // For roi infer task, each roi corresponds to one Parse
  int32_t Parse(
      std::shared_ptr<TrackIdResult> &output,
      std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
      std::shared_ptr<std::vector<hbDNNRoi>> rois,
      std::shared_ptr<NV12PyramidInput> pyramid = nullptr);

  // Get matching statistics
  void PrintStats() const;

 private:
   // Updated to accept float features
   int UpdateReid(
       const std::vector<float> &feature,
       const int roi_idx,
       std::shared_ptr<TrackIdResult> &output);

   // Three-layer matching strategy
  int FastMatch(const std::vector<float> &feature, float &similarity);
  
  // Full scan matching (for small DB optimization)
  int FullScanMatch(const std::vector<float> &feature, float &similarity);
  
  // Calculate similarity with database ID
  float CalculateSimilarityWithId(int id, const std::vector<float> &feature);

  int Query(const float *data,
            std::vector<Item>& target_items);

  int Render(const std::shared_ptr<NV12PyramidInput>& pyramid,
              hbDNNRoi &roi,
              std::string &file_name);

  float CosineSimilarity(const float* data1, const float* data2);

  ItemDatabase db;
  int feature_size_ = 512;
  float threshold_ = 0.75;
  faceid::ModelType model_type_;
  std::unique_ptr<faceid::ModelAdapter> model_adapter_;
  
   // Fast matching components
   std::unique_ptr<faceid::FastFeatureMatcher> fast_matcher_;
   bool use_fast_match_ = true;  // Enable by default
   
   // Dynamic strategy threshold: disable LSH when db_count <= this value
   // For small databases, Cache + Full Scan is faster than LSH overhead
   int fast_match_threshold_ = 50;
};

#endif  // FEATURE_MANAGE_H_
