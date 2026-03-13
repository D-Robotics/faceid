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

#include "include/feature_manage.h"

#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

// Legacy constructor for backward compatibility
FeatureManage::FeatureManage(std::string db_file, int feature_size, float threshold)
    : model_type_(faceid::ModelType::FACEID),
      use_fast_match_(true) {
  feature_size_ = feature_size;
  threshold_ = threshold;
  model_adapter_ = std::make_unique<faceid::ModelAdapter>(model_type_);
  feature_size_ = model_adapter_->GetFeatureDim();  // Use actual feature dim from model
  
  // Initialize fast matcher
  if (use_fast_match_) {
    fast_matcher_ = std::make_unique<faceid::FastFeatureMatcher>(
        feature_size_, threshold_, 8, 16, 5);
  }
  
  db.initialize(db_file);

  if (!db.createTable()) {
    RCLCPP_ERROR(rclcpp::get_logger("faceid_output"),
      "Failed to create table.");
    return;
  }
  
  // Load existing features into LSH index
  if (use_fast_match_) {
    int count = db.getItemCount();
    for (int i = 1; i <= count; ++i) {
      Item item = db.queryItem(i);
      if (!item.feature.empty()) {
        fast_matcher_->AddFeature(item.id, item.feature.data());
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
        "Loaded %d features into fast matcher", count);
  }
}

// New constructor with model type
FeatureManage::FeatureManage(std::string db_file, int feature_size, float threshold,
                             faceid::ModelType model_type)
    : model_type_(model_type),
      use_fast_match_(true) {
  feature_size_ = feature_size;
  threshold_ = threshold;
  model_adapter_ = std::make_unique<faceid::ModelAdapter>(model_type);
  feature_size_ = model_adapter_->GetFeatureDim();  // Use actual feature dim from model
  
  // Initialize fast matcher
  if (use_fast_match_) {
    fast_matcher_ = std::make_unique<faceid::FastFeatureMatcher>(
        feature_size_, threshold_, 8, 16, 5);
  }
  
  db.initialize(db_file);

  if (!db.createTable()) {
    RCLCPP_ERROR(rclcpp::get_logger("faceid_output"),
      "Failed to create table.");
    return;
  }
  
  // Load existing features into LSH index
  if (use_fast_match_) {
    int count = db.getItemCount();
    for (int i = 1; i <= count; ++i) {
      Item item = db.queryItem(i);
      if (!item.feature.empty()) {
        fast_matcher_->AddFeature(item.id, item.feature.data());
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
        "Loaded %d features into fast matcher", count);
  }
}

void FeatureManage::PrintStats() const {
  if (fast_matcher_) {
    auto stats = fast_matcher_->GetStats();
    RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
        "\n=== Fast Matcher Statistics ===\n"
        "Total queries: %zu\n"
        "Cache hits (O(1)): %zu (%.1f%%)\n"
        "LSH hits (O(log n)): %zu (%.1f%%)\n"
        "Full scans (O(n)): %zu (%.1f%%)\n"
        "Average comparisons per query: %.1f",
        stats.total_queries,
        stats.cache_hits, 
        stats.total_queries > 0 ? (100.0 * stats.cache_hits / stats.total_queries) : 0,
        stats.lsh_hits,
        stats.total_queries > 0 ? (100.0 * stats.lsh_hits / stats.total_queries) : 0,
        stats.full_scans,
        stats.total_queries > 0 ? (100.0 * stats.full_scans / stats.total_queries) : 0,
        stats.total_queries > 0 ? 
          (stats.cache_hits * 1.0 + stats.lsh_hits * 5.0 + stats.full_scans * db.getItemCount()) / stats.total_queries 
          : 0);
  }
}

float FeatureManage::CalculateSimilarityWithId(int id, const std::vector<float> &feature) {
  Item item = db.queryItem(id);
  if (item.feature.empty()) {
    return 0.0f;
  }
  return model_adapter_->CalculateSimilarity(item.feature, feature);
}

int FeatureManage::FastMatch(const std::vector<float> &feature, float &similarity) {
  if (!fast_matcher_) {
    // Fallback to original method
    return -1;
  }
  
  auto similarity_func = [this](int id, const float* feat) -> float {
    std::vector<float> query_feat(feat, feat + feature_size_);
    return this->CalculateSimilarityWithId(id, query_feat);
  };
  
  auto get_all_ids_func = [this]() -> std::vector<int> {
    std::vector<int> ids;
    int count = db.getItemCount();
    for (int i = 1; i <= count; ++i) {
      ids.push_back(i);
    }
    return ids;
  };
  
  auto result = fast_matcher_->Match(feature.data(), similarity_func, get_all_ids_func);
  similarity = result.similarity;
  
  // Log which layer was used
  if (result.layer == 1) {
    RCLCPP_DEBUG(rclcpp::get_logger("faceid_output"),
        "Match from CACHE (layer 1): id=%d, sim=%.4f", result.id, result.similarity);
  } else if (result.layer == 2) {
    RCLCPP_DEBUG(rclcpp::get_logger("faceid_output"),
        "Match from LSH (layer 2): id=%d, sim=%.4f", result.id, result.similarity);
  } else if (result.layer == 3) {
    RCLCPP_DEBUG(rclcpp::get_logger("faceid_output"),
        "Match from FULL SCAN (layer 3): id=%d, sim=%.4f", result.id, result.similarity);
  }
  
  return result.id;
}

float FeatureManage::CosineSimilarity(const float* data1, const float* data2) {
  double dot = 0.0;     // 点积
  double norm1 = 0.0;   // |A|^2
  double norm2 = 0.0;   // |B|^2

  for (int i = 0; i < feature_size_; ++i) {
    dot   += static_cast<double>(data1[i]) * data2[i];
    norm1 += static_cast<double>(data1[i]) * data1[i];
    norm2 += static_cast<double>(data2[i]) * data2[i];
  }

  if (norm1 == 0 || norm2 == 0) {
    return 0.0f;
  }

  float denom = std::sqrt(static_cast<float>(norm1))
              * std::sqrt(static_cast<float>(norm2));

  return static_cast<float>(dot) / denom;
}

int32_t FeatureManage::Parse(
    std::shared_ptr<TrackIdResult> &output,
    std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
    std::shared_ptr<std::vector<hbDNNRoi>> rois,
    std::shared_ptr<NV12PyramidInput> pyramid) {

  if (rois == nullptr || static_cast<int>(rois->size()) == 0) {
    RCLCPP_INFO(rclcpp::get_logger("faceid_output"), "get null rois");
    return -1;
  }

  for (size_t i = 0; i < output_tensors.size(); i++) {
    if (!output_tensors[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("faceid_output"), "invalid out tensor");
      return -1;
    }
  }

  std::shared_ptr<TrackIdResult> result = nullptr;
  if (!output) {
    result = std::make_shared<TrackIdResult>();
    result->Reset();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<TrackIdResult>(output);
    result->Reset();
  }
  result->ids.resize(rois->size());

  // Use ModelAdapter to extract features
  auto features = model_adapter_->ExtractFeatures(output_tensors, rois->size());

  for (size_t roi_idx = 0; roi_idx < features.size(); roi_idx++) {
    int ret = UpdateReid(features[roi_idx], roi_idx, result);
    if (pyramid && ret != 0) {
      std::string file_name = std::to_string(ret) + ".jpg";
      Render(pyramid, rois->at(roi_idx), file_name);
    }
  }
  
  // Print stats every 100 frames
  static int frame_count = 0;
  if (++frame_count % 100 == 0) {
    PrintStats();
  }
  
  return 0;
}

int FeatureManage::UpdateReid(
    const std::vector<float> &feature,
    const int roi_idx,
    std::shared_ptr<TrackIdResult> &output) {

  RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
      "\n========== UpdateReid START: roi_idx=%d ==========", roi_idx);

  // Get database count
  int db_count = db.getItemCount();

  // Adaptive threshold
  float adaptive_threshold = threshold_;
  if (db_count <= 3) {
    adaptive_threshold = std::max(threshold_, 0.94f);
  }

  RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
      "Database has %d entries, threshold=%.3f", db_count, adaptive_threshold);

  int ret = 0;
  
  // Try fast match first (three-layer strategy)
  float similarity = 0.0f;
  int best_id = FastMatch(feature, similarity);
  
  if (best_id > 0 && similarity >= adaptive_threshold) {
    // Match found using fast matcher
    output->ids[roi_idx] = best_id;
    
    RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
          "✓ MATCHED (FAST): roi_idx=%d -> id=%d (sim=%.4f, threshold=%.3f)", 
          roi_idx, best_id, similarity, adaptive_threshold);
    
    // Optionally add auxiliary feature
    int feature_count = db.getFeatureCount(best_id);
    if (feature_count < 5) {
      // Simplified: just log, don't add to avoid complexity
      RCLCPP_DEBUG(rclcpp::get_logger("faceid_output"),
          "  Feature count for id=%d: %d", best_id, feature_count);
    }
  } else {
    // No match found - create new ID
    RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
        "No match (best_id=%d, sim=%.4f), creating new ID...", best_id, similarity);

    Item new_item;
    static int face_counter = 0;
    new_item.url = "face_" + std::to_string(++face_counter) + "_" +
                   std::to_string(std::time(nullptr)) + ".jpg";
    new_item.feature = feature;

    if (!db.insertItem(new_item)) {
      RCLCPP_ERROR(rclcpp::get_logger("faceid_output"),
          "Failed to insert item, assigning temporary ID -1");
      output->ids[roi_idx] = -1;
      ret = -1;
    } else {
      // Get new ID
      int newid = db.getItemCount();
      std::vector<Item> latest_items = db.queryItemsByPage(newid - 1, 1);
      if (!latest_items.empty()) {
        newid = latest_items[0].id;
      }

      output->ids[roi_idx] = newid;
      ret = newid;

      // Add to fast matcher
      if (fast_matcher_) {
        fast_matcher_->AddFeature(newid, feature.data());
      }

      RCLCPP_WARN(rclcpp::get_logger("faceid_output"),
        "✓ NEW FACE: roi_idx=%d -> NEW id=%d (database now has %d items)", 
        roi_idx, newid, db_count + 1);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("faceid_output"), 
      "========== UpdateReid END: roi_idx=%d, assigned_id=%d ==========\n", 
      roi_idx, output->ids[roi_idx]);

  return ret;
}

int FeatureManage::Query(const float *data,
                          std::vector<Item>& target_items) {
  int num = db.getItemCount();
  RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
          "Query start, database has %d items, threshold=%.3f", num, threshold_);

  float max_similarity = 0.0f;
  int max_id = -1;

  for (int i = 0; i < num; i++) {
    std::vector<Item> image_items = db.queryItemsByPage(i, 1);

    for (size_t j = 0; j < image_items.size(); j++) {
      auto item = image_items[j];
      if (item.feature.empty()) continue;
      
      std::vector<float> query_feature(data, data + feature_size_);
      item.similarity = model_adapter_->CalculateSimilarity(item.feature, query_feature);

      if (item.similarity > max_similarity) {
        max_similarity = item.similarity;
        max_id = item.id;
      }

      if (item.similarity >= threshold_) {
        target_items.push_back(item);
        RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
          "Match found: id=%d, similarity=%.3f", item.id, item.similarity);
      }
    }
  }

  if (target_items.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
      "No match found, max_similarity=%.3f", max_similarity);
  }

  return 0;
}

int FeatureManage::Render(const std::shared_ptr<NV12PyramidInput>& pyramid,
                           hbDNNRoi &roi,
                           std::string &file_name) {
  // Render function - simplified for now
  return 0;
}
