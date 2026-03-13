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

#include "include/lsh_index.h"

#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

#include "rclcpp/rclcpp.hpp"

namespace faceid {

// ============== LSHIndex Implementation ==============

LSHIndex::LSHIndex(int feature_dim, int num_tables, int hash_bits)
    : feature_dim_(feature_dim),
      num_tables_(num_tables),
      hash_bits_(hash_bits),
      bucket_count_(1ULL << hash_bits),
      tables_(num_tables) {
  InitializeProjections();
  RCLCPP_INFO(rclcpp::get_logger("lsh_index"),
      "LSHIndex initialized: dim=%d, tables=%d, bits=%d, buckets=%zu",
      feature_dim_, num_tables_, hash_bits_, bucket_count_);
}

void LSHIndex::InitializeProjections() {
  projections_.resize(num_tables_);
  std::mt19937 gen(42);  // Fixed seed for reproducibility
  std::normal_distribution<float> dist(0.0f, 1.0f);
  
  for (int t = 0; t < num_tables_; ++t) {
    projections_[t].resize(hash_bits_);
    for (int b = 0; b < hash_bits_; ++b) {
      projections_[t][b].resize(feature_dim_);
      for (int i = 0; i < feature_dim_; ++i) {
        projections_[t][b][i] = dist(gen);
      }
    }
  }
}

uint32_t LSHIndex::ComputeHash(const float* feature, int table_id) const {
  uint32_t hash = 0;
  const auto& table_projections = projections_[table_id];
  
  for (int b = 0; b < hash_bits_; ++b) {
    float dot = 0.0f;
    const auto& proj = table_projections[b];
    for (int i = 0; i < feature_dim_; ++i) {
      dot += feature[i] * proj[i];
    }
    if (dot > 0) {
      hash |= (1U << b);
    }
  }
  return hash;
}

void LSHIndex::Add(int id, const float* feature) {
  for (int t = 0; t < num_tables_; ++t) {
    uint32_t hash = ComputeHash(feature, t);
    tables_[t][hash].push_back({id, feature});
  }
  ++total_entries_;
}

void LSHIndex::Remove(int id, const float* feature) {
  for (int t = 0; t < num_tables_; ++t) {
    uint32_t hash = ComputeHash(feature, t);
    auto& bucket = tables_[t][hash];
    bucket.erase(
      std::remove_if(bucket.begin(), bucket.end(),
        [id, feature](const auto& pair) {
          return pair.first == id && pair.second == feature;
        }),
      bucket.end()
    );
  }
  --total_entries_;
}

std::vector<int> LSHIndex::Query(const float* feature, int top_k) const {
  std::vector<int> all_candidates;
  all_candidates.reserve(num_tables_ * 10);  // Reserve space
  
  // Query all tables
  for (int t = 0; t < num_tables_; ++t) {
    uint32_t hash = ComputeHash(feature, t);
    const auto it = tables_[t].find(hash);
    if (it != tables_[t].end()) {
      for (const auto& [id, _] : it->second) {
        all_candidates.push_back(id);
      }
    }
  }
  
  return GetTopKByFrequency(all_candidates, top_k);
}

std::vector<int> LSHIndex::GetTopKByFrequency(
    const std::vector<int>& candidates, int top_k) const {
  if (candidates.empty()) {
    return {};
  }
  
  // Count frequency
  std::unordered_map<int, int> freq;
  for (int id : candidates) {
    ++freq[id];
  }
  
  // Create frequency pairs
  std::vector<std::pair<int, int>> freq_vec(freq.begin(), freq.end());
  
  // Sort by frequency descending
  std::partial_sort(freq_vec.begin(),
                   freq_vec.begin() + std::min(top_k, static_cast<int>(freq_vec.size())),
                   freq_vec.end(),
                   [](const auto& a, const auto& b) {
                     return a.second > b.second;
                   });
  
  // Extract top-k IDs
  std::vector<int> result;
  result.reserve(std::min(top_k, static_cast<int>(freq_vec.size())));
  for (int i = 0; i < std::min(top_k, static_cast<int>(freq_vec.size())); ++i) {
    result.push_back(freq_vec[i].first);
  }
  
  return result;
}

void LSHIndex::Clear() {
  for (auto& table : tables_) {
    table.clear();
  }
  total_entries_ = 0;
}

// ============== TemporalCache Implementation ==============

TemporalCache::TemporalCache(size_t max_size, float threshold)
    : max_size_(max_size), threshold_(threshold) {}

int TemporalCache::CheckCache(const float* feature, int feature_dim) {
  for (const auto& entry : cache_) {
    float sim = CosineSimilarity(entry.feature.data(), feature, feature_dim);
    if (sim >= threshold_) {
      return entry.id;
    }
  }
  return -1;
}

void TemporalCache::Add(int id, const float* feature, int feature_dim, float similarity) {
  CacheEntry entry;
  entry.id = id;
  entry.feature.assign(feature, feature + feature_dim);
  entry.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  entry.similarity = similarity;
  
  cache_.push_back(std::move(entry));
  
  // Maintain cache size
  while (cache_.size() > max_size_) {
    cache_.pop_front();
  }
}

void TemporalCache::Clear() {
  cache_.clear();
}

float TemporalCache::CosineSimilarity(const float* a, const float* b, int dim) const {
  double dot = 0.0;
  double norm1 = 0.0;
  double norm2 = 0.0;
  
  for (int i = 0; i < dim; ++i) {
    dot += static_cast<double>(a[i]) * b[i];
    norm1 += static_cast<double>(a[i]) * a[i];
    norm2 += static_cast<double>(b[i]) * b[i];
  }
  
  if (norm1 == 0 || norm2 == 0) {
    return 0.0f;
  }
  
  return static_cast<float>(dot / (std::sqrt(norm1) * std::sqrt(norm2)));
}

// ============== FastFeatureMatcher Implementation ==============

FastFeatureMatcher::FastFeatureMatcher(int feature_dim, float threshold,
                                        int lsh_tables, int lsh_bits,
                                        size_t cache_size)
    : feature_dim_(feature_dim),
      threshold_(threshold),
      lsh_index_(feature_dim, lsh_tables, lsh_bits),
      temporal_cache_(cache_size, threshold) {}

void FastFeatureMatcher::AddFeature(int id, const float* feature) {
  lsh_index_.Add(id, feature);
}

void FastFeatureMatcher::RemoveFeature(int id, const float* feature) {
  lsh_index_.Remove(id, feature);
}

FastFeatureMatcher::MatchResult FastFeatureMatcher::Match(
    const float* feature,
    const std::function<float(int, const float*)>& similarity_func,
    const std::function<std::vector<int>()>& get_all_ids_func) {
  
  MatchResult result;
  result.id = -1;
  result.similarity = 0.0f;
  result.layer = 0;
  
  ++stats_.total_queries;
  
  // Layer 1: Temporal Cache (O(1))
  int cached_id = temporal_cache_.CheckCache(feature, feature_dim_);
  if (cached_id > 0) {
    result.id = cached_id;
    result.similarity = 1.0f;  // Already verified by cache
    result.layer = 1;
    ++stats_.cache_hits;
    temporal_cache_.Add(cached_id, feature, feature_dim_, 1.0f);
    return result;
  }
  
  // Layer 2: LSH Index (O(log n))
  auto candidates = lsh_index_.Query(feature, 10);
  if (!candidates.empty()) {
    int best_id = -1;
    float best_sim = 0.0f;
    float second_best_sim = 0.0f;
    
    for (size_t i = 0; i < candidates.size(); ++i) {
      float sim = similarity_func(candidates[i], feature);
      if (sim > best_sim) {
        second_best_sim = best_sim;
        best_sim = sim;
        best_id = candidates[i];
      } else if (sim > second_best_sim) {
        second_best_sim = sim;
      }
    }
    
    // Check if match is valid
    if (best_id > 0 && best_sim >= threshold_) {
      float gap = best_sim - second_best_sim;
      if (gap >= 0.03f || best_sim >= threshold_ + 0.02f) {
        result.id = best_id;
        result.similarity = best_sim;
        result.layer = 2;
        ++stats_.lsh_hits;
        temporal_cache_.Add(best_id, feature, feature_dim_, best_sim);
        return result;
      }
    }
  }
  
  // Layer 3: Full Scan (O(n))
  auto all_ids = get_all_ids_func();
  if (!all_ids.empty()) {
    int best_id = -1;
    float best_sim = 0.0f;
    float second_best_sim = 0.0f;
    
    for (int id : all_ids) {
      float sim = similarity_func(id, feature);
      if (sim > best_sim) {
        second_best_sim = best_sim;
        best_sim = sim;
        best_id = id;
      } else if (sim > second_best_sim) {
        second_best_sim = sim;
      }
    }
    
    if (best_id > 0 && best_sim >= threshold_) {
      float gap = best_sim - second_best_sim;
      if (gap >= 0.03f || best_sim >= threshold_ + 0.02f) {
        result.id = best_id;
        result.similarity = best_sim;
        result.layer = 3;
        ++stats_.full_scans;
        temporal_cache_.Add(best_id, feature, feature_dim_, best_sim);
        return result;
      }
    }
  }
  
  ++stats_.full_scans;
  return result;  // Return -1 for new face
}

void FastFeatureMatcher::Clear() {
  lsh_index_.Clear();
  temporal_cache_.Clear();
  ResetStats();
}

int FastFeatureMatcher::CheckCacheOnly(const float* feature) {
  return temporal_cache_.CheckCache(feature, feature_dim_);
}

}  // namespace faceid
