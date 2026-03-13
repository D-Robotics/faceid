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

#ifndef FACEID_MODEL_ADAPTER_H_
#define FACEID_MODEL_ADAPTER_H_

#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node_data.h"

namespace faceid {

enum class ModelType {
  FACEID,      // Original faceid model (int32 output, 128-dim)
  INSIGHTFACE  // InsightFace model (float output, 512-dim)
};

// Feature extractor interface - handles different data types
class FeatureExtractor {
 public:
  virtual ~FeatureExtractor() = default;
  
  // Get feature dimension
  virtual int GetFeatureDim() const = 0;
  
  // Get element size in bytes
  virtual size_t GetElementSize() const = 0;
  
  // Calculate cosine similarity between two features (native precision)
  virtual float CosineSimilarity(const void* data1, const void* data2) = 0;
  
  // Get tensor data pointer
  virtual const void* GetTensorData(const std::shared_ptr<hobot::dnn_node::DNNTensor>& tensor) = 0;
  
  // Extract feature from tensor at given index
  // Returns vector of bytes (native type), caller must cast appropriately
  virtual std::vector<uint8_t> ExtractFeature(const void* tensor_data, int idx) = 0;
  
  // Convert native feature to float for storage
  virtual std::vector<float> ToFloat(const void* data) = 0;
  
  // Convert float back to native type (for loading from DB)
  virtual std::vector<uint8_t> FromFloat(const std::vector<float>& data) = 0;
};

// FaceID extractor (int32, 128-dim)
class FaceIDExtractor : public FeatureExtractor {
 public:
  FaceIDExtractor() : feature_dim_(128) {}
  
  int GetFeatureDim() const override { return feature_dim_; }
  size_t GetElementSize() const override { return sizeof(int32_t); }
  
  float CosineSimilarity(const void* data1, const void* data2) override;
  const void* GetTensorData(const std::shared_ptr<hobot::dnn_node::DNNTensor>& tensor) override;
  std::vector<uint8_t> ExtractFeature(const void* tensor_data, int idx) override;
  std::vector<float> ToFloat(const void* data) override;
  std::vector<uint8_t> FromFloat(const std::vector<float>& data) override;
  
 private:
  int feature_dim_;
  int stride_ = 4;  // faceid has stride of 4
};

// InsightFace extractor (float, 512-dim)
class InsightFaceExtractor : public FeatureExtractor {
 public:
  InsightFaceExtractor() : feature_dim_(512) {}
  
  int GetFeatureDim() const override { return feature_dim_; }
  size_t GetElementSize() const override { return sizeof(float); }
  
  float CosineSimilarity(const void* data1, const void* data2) override;
  const void* GetTensorData(const std::shared_ptr<hobot::dnn_node::DNNTensor>& tensor) override;
  std::vector<uint8_t> ExtractFeature(const void* tensor_data, int idx) override;
  std::vector<float> ToFloat(const void* data) override;
  std::vector<uint8_t> FromFloat(const std::vector<float>& data) override;
  
 private:
  int feature_dim_;
};

// Unified model adapter
class ModelAdapter {
 public:
  ModelAdapter(ModelType type);
  
  // Extract features from model output tensors
  // Returns float features for storage
  std::vector<std::vector<float>> ExtractFeatures(
      const std::vector<std::shared_ptr<hobot::dnn_node::DNNTensor>>& output_tensors,
      int num_rois);
  
  // Calculate similarity using native precision
  float CalculateSimilarity(const std::vector<float>& data1, 
                            const std::vector<float>& data2);
  
  // Get feature dimension
  int GetFeatureDim() const { return extractor_->GetFeatureDim(); }
  
  // Get model type
  ModelType GetModelType() const { return model_type_; }
  
  // Get feature extractor
  FeatureExtractor* GetExtractor() { return extractor_.get(); }
  
  // Utility functions
  static std::string ModelTypeToString(ModelType type);
  static ModelType StringToModelType(const std::string& str);
  
 private:
  ModelType model_type_;
  std::unique_ptr<FeatureExtractor> extractor_;
};

}  // namespace faceid

#endif  // FACEID_MODEL_ADAPTER_H_
