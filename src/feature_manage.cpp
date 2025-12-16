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

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"

#include "include/feature_manage.h"

float cosine_similarity(const int32_t* data1, const int32_t* data2) {
    int64_t dot = 0;     // 点积
    int64_t norm1 = 0;   // |A|^2
    int64_t norm2 = 0;   // |B|^2

    for (int i = 0; i < 128; ++i) {
        dot   += static_cast<int64_t>(data1[i]) * data2[i];
        norm1 += static_cast<int64_t>(data1[i]) * data1[i];
        norm2 += static_cast<int64_t>(data2[i]) * data2[i];
    }

    if (norm1 == 0 || norm2 == 0) {
        return 0.0f;
    }

    float denom = std::sqrt(static_cast<float>(norm1))
                * std::sqrt(static_cast<float>(norm2));

    return static_cast<float>(dot) / denom;
}

FeatureManage::FeatureManage(std::string db_file, int feature_size, float threshold) {
  feature_size_ = feature_size;
  threshold_ = threshold;
  db.initialize(db_file);

  if (!db.createTable()) {
    RCLCPP_ERROR(rclcpp::get_logger("faceid_output"),
      "Failed to create table.");
    return;
  }
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

  for (int i = 0; i < output_tensors.size(); i++) {
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

  output_tensors[0]->CACHE_INVALIDATE();
  int32_t* data = output_tensors[0]->GetTensorData<int32_t>();
  // 取对应的float_tensor解析
  for (int roi_idx = 0; roi_idx < static_cast<int>(rois->size()); roi_idx++) {
    std::vector<int32_t> feature(feature_size_);
    int stride = 4;
    for (size_t i = 0; i < feature_size_; ++i) {
      feature[i] = data[roi_idx * feature_size_ * stride + i * stride];  // stride = 4
    }
    int ret = UpdateReid(feature, roi_idx, result);
    if (pyramid && ret != 0) {
      std::string file_name = std::to_string(ret) + ".jpg";
      Render(pyramid, rois->at(roi_idx), file_name);
    }
  }
  return 0;
}

int FeatureManage::UpdateReid(
    std::vector<int32_t> &feature,
    const int roi_idx,
    std::shared_ptr<TrackIdResult> &output) {

  std::vector<Item> target_items;
  int ret = Query(feature.data(), target_items);

  if (target_items.size() == 1) {
    output->ids[roi_idx] = target_items[0].id;
    RCLCPP_INFO(rclcpp::get_logger("faceid_output"),
          "Query success, result: %d.", target_items[0].id);
  } else {    
    int num = db.getItemCount();
    int newid = num + 1;
    output->ids[roi_idx] = newid;

    Item Item;
    Item.url = std::to_string(newid) + ".jpg";
    Item.feature.assign(feature.begin(), feature.end()); // 复制特征值
    if (!db.insertItem(Item)) {
      RCLCPP_ERROR(rclcpp::get_logger("faceid_output"), "Failed to insert item.");
    }
    ret = newid;
    RCLCPP_WARN(rclcpp::get_logger("faceid_output"),
      "Query failed, storage: %d.", newid);
  }
  return ret;
}


// 定义函数，过滤出 type 为 false 的 Item
int FeatureManage::Query(const int32_t *data,
              std::vector<Item>& target_items) {

  int num = db.getItemCount();
  RCLCPP_DEBUG(rclcpp::get_logger("faceid_output"),
          "Query start, num of database: %d.", num);
  // int page_num = 10;
  int page_num = 1;
  // num = (num / page_num) + num % page_num != 0 ? 1: 0;
  num = (num / page_num) + 1;

  std::stringstream sss;
  for (int i = 0; i < num; i++) {
    std::vector<Item> image_items = db.queryItemsByPage(i, page_num);
    
    for (int j = 0; j < image_items.size(); j++) {
      auto item = image_items[j];
      const int32_t* data_image = item.feature.data();
      item.similarity = cosine_similarity(data_image, data);
      std::stringstream ss;
      ss << "id: " << page_num * i + j
          << ", item similarity: " << item.similarity;
      RCLCPP_INFO(rclcpp::get_logger("faceid_output"), "%s", ss.str().c_str());
      sss << "\nid: " << page_num * i + j
          << ", item similarity: " << item.similarity;
      if (item.similarity < threshold_) {
        continue;
      }
      target_items.push_back(item);
    }
  }

  // 降序排序
  std::sort(target_items.begin(), target_items.end(), compareBySimilarity);

  // 检查向量大小并删除多余元素
  if (target_items.size() > 1) {
    target_items.erase(target_items.begin() + 1, target_items.end());
  } else if (target_items.size() == 0){
    RCLCPP_WARN(rclcpp::get_logger("faceid_output"), "%s", sss.str().c_str());
  }

  return 0;
}

int FeatureManage::Render(const std::shared_ptr<NV12PyramidInput>& pyramid,
                           hbDNNRoi &roi,
                           std::string &file_name) {

  char* y_img = reinterpret_cast<char*>(pyramid->y_vir_addr);
  char* uv_img = reinterpret_cast<char*>(pyramid->uv_vir_addr);
  auto height = pyramid->height;
  auto width = pyramid->width;
  auto img_y_size = height * width;
  auto img_uv_size = img_y_size / 2;
  char* buf = new char[img_y_size + img_uv_size];
  memcpy(buf, y_img, img_y_size);
  memcpy(buf + img_y_size, uv_img, img_uv_size);
  cv::Mat nv12(height * 3 / 2, width, CV_8UC1, buf);
  cv::Mat bgr;
  cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
  delete[] buf;

  cv::Rect roi_rect(roi.left, roi.top, roi.right - roi.left, roi.bottom - roi.top);
  cv::Mat roi_img = bgr(roi_rect);
  cv::imwrite(file_name, roi_img);

  return 0;
}