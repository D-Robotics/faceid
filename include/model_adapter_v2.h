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

// Feature extractor interface
class FeatureExtractor {
 public:
  virtual ~FeatureExtractor() = default;
  
  // Calculate cosine similarity between two features
  virtual float CosineSimilarity(const void* data1, const void* data2, int dim) = 0;
  
  // Get feature size in bytes
  virtual size_t GetFeatureSize(int dim) = 0;
  
  // Get data pointer from tensor
  virtual const void* GetTensorData(const std::shared_ptr<hobot::dnn_node::DNNTensor>& tensor) = 0;
  
  // Copy feature from tensor
  virtual void CopyFeature(const void* src, void* dst, int idx, int dim) = 0;
  
  // Convert to float for storage
  virtual std::vector<float> ToFloat(const void* data, int dim) = 0;
};

// FaceID extractor (int32)
class FaceIDExtractor : public FeatureExtractor {
 public:
  float CosineSimilarity(const void* data1, const void* data2, int dim) override;
  size_t GetFeatureSize(int dim) override { return dim * sizeof(int32_t); }
  const void* GetTensorData(const std::shared_ptr<hobot::dnn_node::DNNTensor>& tensor) override;
  void CopyFeature(const void* src, void* dst, int idx, int dim) override;
  std::vector<float> ToFloat(const void* data, int dim) override;
};

// InsightFace extractor (float)
class InsightFaceExtractor : public FeatureExtractor {
 public:
  float CosineSimilarity(const void* data1, const void* data2, int dim) override;
  size_t GetFeatureSize(int dim) override { return dim * sizeof(float); }
  const void* GetTensorData(const std::shared_ptr<hobot::dnn_node::DNNTensor>& tensor) override;
  void CopyFeature(const void* src, void* dst, int idx, int dim) override;
  std::vector<float> ToFloat(const void* data, int dim) override;
};

class ModelAdapter {
 public:
  ModelAdapter(ModelType type);
  
  // Extract features from model output tensors
  // Returns vector of features (as float for storage, but calculated with native precision)
  std::vector<std::vector<float>> ExtractFeatures(
      const std::vector<std::shared_ptr<hobot::dnn_node::DNNTensor>>& output_tensors,
      int num_rois);
  
  // Calculate similarity with native precision
  float CalculateSimilarity(const float* data1, const float* data2);
  
  // Get feature dimension
  int GetFeatureDim() const { return feature_dim_; }
  
  // Get model type
  ModelType GetModelType() const { return model_type_; }
  
  // Get feature extractor
  FeatureExtractor* GetExtractor() { return extractor_.get(); }
  
  // Get model type string
  static std::string ModelTypeToString(ModelType type);
  static ModelType StringToModelType(const std::string& str);
  
 private:
  ModelType model_type_;
  int feature_dim_;
  std::unique_ptr<FeatureExtractor> extractor_;
};

}  // namespace faceid

#endif  // FACEID_MODEL_ADAPTER_H_
