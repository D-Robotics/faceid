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

#include "include/model_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "rclcpp/rclcpp.hpp"

namespace faceid {

// ============== FaceIDExtractor (int32, 128-dim) ==============

float FaceIDExtractor::CosineSimilarity(const void* data1, const void* data2) {
  const int32_t* d1 = static_cast<const int32_t*>(data1);
  const int32_t* d2 = static_cast<const int32_t*>(data2);
  
  int64_t dot = 0;
  int64_t norm1 = 0;
  int64_t norm2 = 0;

  for (int i = 0; i < feature_dim_; ++i) {
    dot   += static_cast<int64_t>(d1[i]) * d2[i];
    norm1 += static_cast<int64_t>(d1[i]) * d1[i];
    norm2 += static_cast<int64_t>(d2[i]) * d2[i];
  }

  if (norm1 == 0 || norm2 == 0) {
    return 0.0f;
  }

  float denom = std::sqrt(static_cast<float>(norm1))
              * std::sqrt(static_cast<float>(norm2));

  return static_cast<float>(dot) / denom;
}

const void* FaceIDExtractor::GetTensorData(
    const std::shared_ptr<hobot::dnn_node::DNNTensor>& tensor) {
  return static_cast<const void*>(tensor->GetTensorData<int32_t>());
}

std::vector<uint8_t> FaceIDExtractor::ExtractFeature(const void* tensor_data, int idx) {
  const int32_t* data = static_cast<const int32_t*>(tensor_data);
  std::vector<uint8_t> feature(feature_dim_ * sizeof(int32_t));
  int32_t* feature_ptr = reinterpret_cast<int32_t*>(feature.data());
  
  for (int i = 0; i < feature_dim_; ++i) {
    feature_ptr[i] = data[idx * feature_dim_ * stride_ + i * stride_];
  }
  
  return feature;
}

std::vector<float> FaceIDExtractor::ToFloat(const void* data) {
  const int32_t* d = static_cast<const int32_t*>(data);
  std::vector<float> result(feature_dim_);
  for (int i = 0; i < feature_dim_; ++i) {
    result[i] = static_cast<float>(d[i]);
  }
  return result;
}

std::vector<uint8_t> FaceIDExtractor::FromFloat(const std::vector<float>& data) {
  std::vector<uint8_t> result(feature_dim_ * sizeof(int32_t));
  int32_t* ptr = reinterpret_cast<int32_t*>(result.data());
  for (int i = 0; i < feature_dim_; ++i) {
    ptr[i] = static_cast<int32_t>(data[i]);
  }
  return result;
}

// ============== InsightFaceExtractor (float, 512-dim) ==============

float InsightFaceExtractor::CosineSimilarity(const void* data1, const void* data2) {
  const float* d1 = static_cast<const float*>(data1);
  const float* d2 = static_cast<const float*>(data2);
  
  double dot = 0.0;
  double norm1 = 0.0;
  double norm2 = 0.0;

  for (int i = 0; i < feature_dim_; ++i) {
    dot   += static_cast<double>(d1[i]) * d2[i];
    norm1 += static_cast<double>(d1[i]) * d1[i];
    norm2 += static_cast<double>(d2[i]) * d2[i];
  }

  if (norm1 == 0 || norm2 == 0) {
    return 0.0f;
  }

  float denom = std::sqrt(static_cast<float>(norm1))
              * std::sqrt(static_cast<float>(norm2));

  return static_cast<float>(dot) / denom;
}

const void* InsightFaceExtractor::GetTensorData(
    const std::shared_ptr<hobot::dnn_node::DNNTensor>& tensor) {
  return static_cast<const void*>(tensor->GetTensorData<float>());
}

std::vector<uint8_t> InsightFaceExtractor::ExtractFeature(const void* tensor_data, int idx) {
  const float* data = static_cast<const float*>(tensor_data);
  std::vector<uint8_t> feature(feature_dim_ * sizeof(float));
  float* feature_ptr = reinterpret_cast<float*>(feature.data());
  
  for (int i = 0; i < feature_dim_; ++i) {
    feature_ptr[i] = data[idx * feature_dim_ + i];
  }
  
  return feature;
}

std::vector<float> InsightFaceExtractor::ToFloat(const void* data) {
  const float* d = static_cast<const float*>(data);
  return std::vector<float>(d, d + feature_dim_);
}

std::vector<uint8_t> InsightFaceExtractor::FromFloat(const std::vector<float>& data) {
  std::vector<uint8_t> result(feature_dim_ * sizeof(float));
  float* ptr = reinterpret_cast<float*>(result.data());
  std::memcpy(ptr, data.data(), feature_dim_ * sizeof(float));
  return result;
}

// ============== ModelAdapter ==============

ModelAdapter::ModelAdapter(ModelType type) : model_type_(type) {
  switch (model_type_) {
    case ModelType::FACEID:
      extractor_ = std::make_unique<FaceIDExtractor>();
      break;
    case ModelType::INSIGHTFACE:
      extractor_ = std::make_unique<InsightFaceExtractor>();
      break;
  }
  RCLCPP_INFO(rclcpp::get_logger("faceid"),
      "ModelAdapter initialized: type=%s, feature_dim=%d",
      ModelTypeToString(model_type_).c_str(), GetFeatureDim());
}

std::vector<std::vector<float>> ModelAdapter::ExtractFeatures(
    const std::vector<std::shared_ptr<hobot::dnn_node::DNNTensor>>& output_tensors,
    int num_rois) {
  std::vector<std::vector<float>> features;
  
  if (output_tensors.empty() || !output_tensors[0]) {
    RCLCPP_ERROR(rclcpp::get_logger("faceid"), "Invalid output tensor");
    return features;
  }
  
  output_tensors[0]->CACHE_INVALIDATE();
  const void* tensor_data = extractor_->GetTensorData(output_tensors[0]);
  
  for (int roi_idx = 0; roi_idx < num_rois; roi_idx++) {
    // Extract native feature
    std::vector<uint8_t> native_feature = extractor_->ExtractFeature(tensor_data, roi_idx);
    // Convert to float for storage
    std::vector<float> float_feature = extractor_->ToFloat(native_feature.data());
    features.push_back(std::move(float_feature));
  }
  
  return features;
}

float ModelAdapter::CalculateSimilarity(const std::vector<float>& data1, 
                                         const std::vector<float>& data2) {
  // Convert back to native type for accurate calculation
  std::vector<uint8_t> native1 = extractor_->FromFloat(data1);
  std::vector<uint8_t> native2 = extractor_->FromFloat(data2);
  
  // Calculate using native precision
  return extractor_->CosineSimilarity(native1.data(), native2.data());
}

std::string ModelAdapter::ModelTypeToString(ModelType type) {
  switch (type) {
    case ModelType::FACEID:
      return "faceid";
    case ModelType::INSIGHTFACE:
      return "insightface";
    default:
      return "unknown";
  }
}

ModelType ModelAdapter::StringToModelType(const std::string& str) {
  if (str == "insightface" || str == "INSIGHTFACE") {
    return ModelType::INSIGHTFACE;
  }
  return ModelType::FACEID;  // default
}

}  // namespace faceid
