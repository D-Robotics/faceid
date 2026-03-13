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

#ifndef LSH_INDEX_H_
#define LSH_INDEX_H_

#include <vector>
#include <deque>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <algorithm>
#include <functional>
#include <cstdint>

namespace faceid {

// LSH (Locality Sensitive Hashing) Index for fast approximate nearest neighbor search
// Supports multi-probe LSH with multiple hash tables
class LSHIndex {
 public:
  // Constructor: 
  //   feature_dim: feature vector dimension (128 or 512)
  //   num_tables: number of hash tables (default: 8)
  //   hash_bits: number of bits per hash (default: 16)
  LSHIndex(int feature_dim, int num_tables = 8, int hash_bits = 16);
  
  // Add a feature vector to the index
  //   id: face ID
  //   feature: feature vector pointer
  void Add(int id, const float* feature);
  
  // Remove a feature vector from the index
  void Remove(int id, const float* feature);
  
  // Query for candidate IDs
  //   feature: query feature vector
  //   top_k: maximum number of candidates to return (default: 10)
  // Returns: list of candidate IDs sorted by occurrence frequency
  std::vector<int> Query(const float* feature, int top_k = 10) const;
  
  // Clear all entries
  void Clear();
  
  // Get number of tables
  int GetNumTables() const { return num_tables_; }
  
  // Get hash bits
  int GetHashBits() const { return hash_bits_; }
  
  // Get bucket count per table
  size_t GetBucketCount() const { return 1ULL << hash_bits_; }
  
  // Get total entry count
  size_t GetEntryCount() const { return total_entries_; }
  
 private:
  int feature_dim_;          // Feature dimension (128 or 512)
  int num_tables_;           // Number of hash tables
  int hash_bits_;            // Bits per hash value
  size_t bucket_count_;      // 2^hash_bits
  size_t total_entries_ = 0; // Total entries in index
  
  // Random projection vectors for each table
  // Shape: [num_tables][hash_bits][feature_dim]
  std::vector<std::vector<std::vector<float>>> projections_;
  
  // Hash tables: table_id -> {bucket_hash -> {id, feature_ptr}}
  std::vector<std::unordered_map<uint32_t, std::vector<std::pair<int, const float*>>>> tables_;
  
  // Compute hash for a feature vector in a specific table
  uint32_t ComputeHash(const float* feature, int table_id) const;
  
  // Generate random projection vectors
  void InitializeProjections();
  
  // Get top-k IDs by frequency
  std::vector<int> GetTopKByFrequency(const std::vector<int>& candidates, int top_k) const;
};

// Temporal cache for fast consecutive frame matching
class TemporalCache {
 public:
  struct CacheEntry {
    int id;
    std::vector<float> feature;
    int64_t timestamp;
    float similarity;
  };
  
  TemporalCache(size_t max_size = 5, float threshold = 0.75);
  
  // Check if feature matches any cached entry
  // Returns: matched ID if found, -1 otherwise
  int CheckCache(const float* feature, int feature_dim);
  
  // Add a new entry to cache
  void Add(int id, const float* feature, int feature_dim, float similarity);
  
  // Clear all cache entries
  void Clear();
  
  // Get cache size
  size_t Size() const { return cache_.size(); }
  
  // Get max cache size
  size_t GetMaxSize() const { return max_size_; }
  
 private:
  size_t max_size_;
  float threshold_;
  std::deque<CacheEntry> cache_;
  
  // Cosine similarity calculation
  float CosineSimilarity(const float* a, const float* b, int dim) const;
};

// Fast feature matcher combining LSH and Temporal Cache
class FastFeatureMatcher {
 public:
  FastFeatureMatcher(int feature_dim, float threshold, 
                     int lsh_tables = 8, int lsh_bits = 16,
                     size_t cache_size = 5);
  
  // Add feature to index
  void AddFeature(int id, const float* feature);
  
  // Remove feature from index
  void RemoveFeature(int id, const float* feature);
  
  // Fast match with three-layer strategy
  // Returns: matched ID or -1 for new face
  struct MatchResult {
    int id;
    float similarity;
    int layer;  // 1: cache, 2: LSH, 3: full scan
  };
  
  MatchResult Match(const float* feature, 
                   const std::function<float(int, const float*)>& similarity_func,
                   const std::function<std::vector<int>()>& get_all_ids_func);
  
  // Clear all data
  void Clear();
  
  // Get statistics
  struct Stats {
    size_t cache_hits = 0;
    size_t lsh_hits = 0;
    size_t full_scans = 0;
    size_t total_queries = 0;
  };
  Stats GetStats() const { return stats_; }
  void ResetStats() { stats_ = Stats(); }
  
 private:
  int feature_dim_;
  float threshold_;
  LSHIndex lsh_index_;
  TemporalCache temporal_cache_;
  Stats stats_;
};

}  // namespace faceid

#endif  // LSH_INDEX_H_
