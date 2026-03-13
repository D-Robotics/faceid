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

#include "include/faceid_node.h"

#include <math.h>
#include <unistd.h>

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "include/ai_msg_manage.h"
#include "include/feature_manage.h"
#include "rclcpp/rclcpp.hpp"

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"

builtin_interfaces::msg::Time ConvertToRosTime(
    const struct timespec& time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start,
                      const builtin_interfaces::msg::Time& end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}

FaceidNode::FaceidNode(const std::string& node_name,
                                const NodeOptions& options)
    : DnnNode(node_name, options) {

  float threshold = 0.70;

  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<float>("threshold", threshold);
  this->declare_parameter<std::string>("db_file", db_file_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name",
                                       ai_msg_pub_topic_name_);
  this->declare_parameter<std::string>("ai_msg_sub_topic_name",
                                       ai_msg_sub_topic_name_);
  this->declare_parameter<std::string>("ros_img_topic_name",
                                       ros_img_topic_name_);
  // NEW: Model type parameter
  this->declare_parameter<std::string>("model_type", model_type_str_);

  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<float>("threshold", threshold);
  this->get_parameter<std::string>("db_file", db_file_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name",
                                   ai_msg_pub_topic_name_);
  this->get_parameter<std::string>("ai_msg_sub_topic_name",
                                   ai_msg_sub_topic_name_);
  this->get_parameter<std::string>("ros_img_topic_name",
                                   ros_img_topic_name_);
  // NEW: Get model type
  this->get_parameter<std::string>("model_type", model_type_str_);
  model_type_ = faceid::ModelAdapter::StringToModelType(model_type_str_);
  
  // Set model-specific defaults
  if (model_type_ == faceid::ModelType::INSIGHTFACE) {
    feature_dim_ = 512;
    if (model_file_name_ == "config/faceID.hbm") {
      // Use default insightface model if not specified
      model_file_name_ = "config/insightface.bin";
    }
  } else {
    feature_dim_ = 128;
    if (model_file_name_ == "config/faceID.hbm") {
      // Keep default faceid model (.hbm format) - path is correct
      // model_file_name_ stays as "config/faceID.hbm"
    }
  }
  
  std::stringstream ss;
  ss << "Parameter:"
     << "\n feed_type(0:local, 1:sub): " << feed_type_
     << "\n db_file: " << db_file_
     << "\n model_file_name: " << model_file_name_
     << "\n model_type: " << model_type_str_
     << "\n feature_dim: " << feature_dim_
     << "\n dump_render_img: " << dump_render_img_
     << "\n is_sync_mode: " << is_sync_mode_
     << "\n is_shared_mem_sub: " << is_shared_mem_sub_
     << "\n threshold: " << threshold
     << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_
     << "\n ai_msg_sub_topic_name: " << ai_msg_sub_topic_name_
     << "\n ros_img_topic_name: " << ros_img_topic_name_;

#ifdef SHARED_MEM_ENABLED
  this->declare_parameter<std::string>("sharedmem_img_topic_name",
                                       sharedmem_img_topic_name_);
  this->get_parameter<std::string>("sharedmem_img_topic_name",
                                   sharedmem_img_topic_name_);
  ss << "\n sharedmem_img_topic_name: " << sharedmem_img_topic_name_;
#endif

  RCLCPP_WARN(this->get_logger(), "%s", ss.str().c_str());

  if (Init() != 0) {
    RCLCPP_ERROR(this->get_logger(), "Init failed!");
  }

  // 未指定模型名，从加载的模型中查询出模型名
  if (model_name_.empty()) {
    if (!GetModel()) {
      RCLCPP_ERROR(rclcpp::get_logger("faceid"), "Get model fail.");
    } else {
      model_name_ = GetModel()->GetName();
      RCLCPP_WARN(rclcpp::get_logger("faceid"), "Get model name: %s from load model.", model_name_.c_str());
    }
  }

  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Get model input size fail!");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "The model input width is %d and height is %d",
                model_input_width_,
                model_input_height_);
  }

  // 删除之前的数据库
  // std::string command = "rm -rf " + db_file_;
  // std::system(command.c_str());
  // command = "rm /mnt/testdata/*.jpg";
  // std::system(command.c_str());

  feature_manage_ = std::make_shared<FeatureManage>(db_file_, feature_dim_, threshold, model_type_);

  if (0 == feed_type_) {
    FeedFromLocal();
  } else {
    predict_task_ = std::make_shared<std::thread>(
        std::bind(&FaceidNode::RunPredict, this));

    ai_msg_manage_ = std::make_shared<AiMsgManage>();

    RCLCPP_INFO(this->get_logger(),
                "ai_msg_pub_topic_name_: %s",
                ai_msg_pub_topic_name_.data());
    msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        ai_msg_pub_topic_name_, 10);

    RCLCPP_INFO(this->get_logger(),
                "Create subscription with topic_name: %s",
                ai_msg_sub_topic_name_.c_str());
    ai_msg_subscription_ =
        this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            ai_msg_sub_topic_name_,
            10,
            std::bind(
                &FaceidNode::AiMsgProcess, this, std::placeholders::_1));

    if (is_shared_mem_sub_) {
#ifdef SHARED_MEM_ENABLED
      RCLCPP_WARN(this->get_logger(),
                  "Create hbmem_subscription with topic_name: %s",
                  sharedmem_img_topic_name_.c_str());
      sharedmem_img_subscription_ =
          this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
              sharedmem_img_topic_name_,
              rclcpp::SensorDataQoS(),
              std::bind(&FaceidNode::SharedMemImgProcess,
                        this,
                        std::placeholders::_1));
#else
      RCLCPP_ERROR(this->get_logger(), "Unsupport shared mem");
#endif
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Create subscription with topic_name: %s",
                  ros_img_topic_name_.c_str());
      ros_img_subscription_ =
          this->create_subscription<sensor_msgs::msg::Image>(
              ros_img_topic_name_,
              10,
              std::bind(
                  &FaceidNode::RosImgProcess, this, std::placeholders::_1));
    }
  }
}

FaceidNode::~FaceidNode() {
  std::unique_lock<std::mutex> lg(mtx_img_);
  cv_img_.notify_all();
  lg.unlock();

  if (predict_task_ && predict_task_->joinable()) {
    predict_task_->join();
    predict_task_.reset();
  }
}

int FaceidNode::SetNodePara() {
  RCLCPP_INFO(this->get_logger(), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 4;
  return 0;
}

int FaceidNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  if (!node_output) {
    RCLCPP_ERROR(this->get_logger(), "Invalid node output");
    return -1;
  }

  if (!msg_publisher_ && feed_type_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid msg publisher");
    return -1;
  }

  auto faceid_output = std::dynamic_pointer_cast<FeatOutput>(node_output);
  if (!faceid_output) {
    return -1;
  }

  if (faceid_output->valid_rois == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Invalid Valid Rois");
    return -1;
  }

  if (node_output->rt_stat->fps_updated) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "input fps: %.2f, out fps: %.2f, "
        "infer time ms: %d, post process time ms: %d",
        node_output->rt_stat->input_fps,
        node_output->rt_stat->output_fps,
        node_output->rt_stat->infer_time_ms,
        node_output->rt_stat->parse_time_ms);
  }

  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);

  // 1. 解析模型输出向量
  auto reid_val = std::make_shared<TrackIdResult>();
  int ret = feature_manage_->Parse(reid_val, faceid_output->output_tensors, faceid_output->valid_rois, faceid_output->pyramid);

  if (feed_type_ == 0) {
    return 0;
  }

  if (ret != 0) {
    msg_publisher_->publish(std::move(faceid_output->ai_msg));
    return -1;
  }
  
  if (!reid_val) {
    RCLCPP_ERROR(this->get_logger(), "Invalid faceid val.");
    return -1;
  }

  if (faceid_output->image_msg_header && faceid_output->valid_rois) {
    std::stringstream ss;
    ss << "Output from, frame_id: " << faceid_output->image_msg_header->frame_id
       << ", stamp: " << faceid_output->image_msg_header->stamp.sec << "_"
       << faceid_output->image_msg_header->stamp.nanosec
       << ", face rois size: " << faceid_output->valid_rois->size()
       << ", face rois idx size: " << faceid_output->valid_roi_idx.size()
       << ", person ids size: " << reid_val->ids.size();
       RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  // 本地模式不需要发布推理话题消息
  if (feed_type_ == 0) {
    return 0;
  }
  
  // 3. 发布模型推理话题消息
  ai_msgs::msg::PerceptionTargets::UniquePtr& msg = faceid_output->ai_msg;
  if (!msg) {
    RCLCPP_ERROR(this->get_logger(), "Invalid ai msg!");
    msg_publisher_->publish(std::move(faceid_output->ai_msg));
    return -1;
  }

  if (reid_val->ids.size() != faceid_output->valid_rois->size() ||
      faceid_output->valid_rois->size() !=
          faceid_output->valid_roi_idx.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Check faceid fail");
    msg_publisher_->publish(std::move(msg));
    return 0;
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(
      new ai_msgs::msg::PerceptionTargets());
  ai_msg->set__header(msg->header);
  ai_msg->set__disappeared_targets(msg->disappeared_targets);

  if (node_output->rt_stat) {
    ai_msg->set__fps(round(node_output->rt_stat->output_fps));
  }

  const std::map<size_t, size_t>& valid_roi_idx =
      faceid_output->valid_roi_idx;
  int face_roi_idx = 0;
  for (const auto& in_target : msg->targets) {
    // 缓存target的 等kps
    std::vector<ai_msgs::msg::Point> tar_points;
    for (const auto& pts : in_target.points) {
      tar_points.push_back(pts);
    }

    ai_msgs::msg::Target target;
    target.set__type(in_target.type);
    target.set__rois(in_target.rois);
    target.set__attributes(in_target.attributes);
    target.set__captures(in_target.captures);
    if ("face" == in_target.rois[0].type) {
      if (valid_roi_idx.find(face_roi_idx) == valid_roi_idx.end()) {
        continue;
      }

      auto body_valid_roi_idx = valid_roi_idx.at(face_roi_idx);
      if (body_valid_roi_idx >= reid_val->ids.size()) {
        RCLCPP_ERROR(this->get_logger(),
                      "faceid outputs %d unmatch with roi idx %d",
                      reid_val->ids.size(),
                      body_valid_roi_idx);
        break;
      }

      target.set__track_id(reid_val->ids[body_valid_roi_idx]);
      face_roi_idx++;
    }

    target.set__points(tar_points);
    ai_msg->targets.emplace_back(target);
  }

  for (const auto& target : ai_msg->targets) {
    std::stringstream ss;
    ss << "target id: " << target.track_id
        << ", rois size: " << target.rois.size()
        << " points size: " << target.points.size() << " ";
    if (!target.rois.empty()) {
      ss << " roi type: " << target.rois.front().type << " ";
    }
    if (!target.points.empty()) {
      ss << " point type: " << target.points.front().type << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }
  
  // 4. 记录 ai msg 的输出帧率
  ai_msg->set__perfs(msg->perfs);

  faceid_output->perf_preprocess.set__time_ms_duration(
      CalTimeMsDuration(faceid_output->perf_preprocess.stamp_start,
                        faceid_output->perf_preprocess.stamp_end));
  ai_msg->perfs.push_back(faceid_output->perf_preprocess);

  // predict
  if (faceid_output->rt_stat) {
    ai_msgs::msg::Perf perf;
    perf.set__type(model_name_ + "_predict_infer");
    perf.set__stamp_start(
        ConvertToRosTime(faceid_output->rt_stat->infer_timespec_start));
    perf.set__stamp_end(
        ConvertToRosTime(faceid_output->rt_stat->infer_timespec_end));
    perf.set__time_ms_duration(faceid_output->rt_stat->infer_time_ms);
    ai_msg->perfs.push_back(perf);

    perf.set__type(model_name_ + "_predict_parse");
    perf.set__stamp_start(
        ConvertToRosTime(faceid_output->rt_stat->parse_timespec_start));
    perf.set__stamp_end(
        ConvertToRosTime(faceid_output->rt_stat->parse_timespec_end));
    perf.set__time_ms_duration(faceid_output->rt_stat->parse_time_ms);
    ai_msg->perfs.push_back(perf);
  }

  ai_msgs::msg::Perf perf_postprocess;
  perf_postprocess.set__type(model_name_ + "_postprocess");
  perf_postprocess.set__stamp_start(ConvertToRosTime(time_now));
  clock_gettime(CLOCK_REALTIME, &time_now);
  perf_postprocess.set__stamp_end(ConvertToRosTime(time_now));
  perf_postprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_postprocess.stamp_start, perf_postprocess.stamp_end));
  ai_msg->perfs.emplace_back(perf_postprocess);

  // 从发布图像到发布AI结果的延迟
  ai_msgs::msg::Perf perf_pipeline;
  perf_pipeline.set__type(model_name_ + "_pipeline");
  perf_pipeline.set__stamp_start(ai_msg->header.stamp);
  perf_pipeline.set__stamp_end(perf_postprocess.stamp_end);
  perf_pipeline.set__time_ms_duration(
      CalTimeMsDuration(perf_pipeline.stamp_start, perf_pipeline.stamp_end));
  ai_msg->perfs.push_back(perf_pipeline);

  msg_publisher_->publish(std::move(ai_msg));

  return 0;
}

void FaceidNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "_"
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<NV12PyramidInput> pyramid = nullptr;
  if ("nv12" == img_msg->encoding) {
    // std::string fname = "img_" + std::to_string(img_msg->header.stamp.sec)
    //  + "." + std::to_string(img_msg->header.stamp.nanosec) + ".nv12";
    // std::ofstream ofs(fname);
    // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
    //   img_msg->data.size());

    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        img_msg->height,
        img_msg->width);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupport img encoding: %s",
    img_msg->encoding.data());
  }
  if (!pyramid) {
    RCLCPP_ERROR(this->get_logger(), "Get Nv12 pym fail");
    return;
  }
  
  RCLCPP_WARN(this->get_logger(), "prepare input");
  // 2. 创建推理输出数据
  auto dnn_output = std::make_shared<FeatOutput>();
  // 将图片消息的header填充到输出数据中，用于表示推理输出对应的输入信息
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);
  // 将当前的时间戳填充到输出数据中，用于计算perf
  dnn_output->perf_preprocess.stamp_start.sec = time_start.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_start.tv_nsec;
  dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
  if (dump_render_img_) {
    dnn_output->pyramid = pyramid;
  }
  dnn_output->img_h = img_msg->height;
  dnn_output->img_w = img_msg->width;

  // 3. 将准备好的输入输出数据存进缓存
  std::unique_lock<std::mutex> lg(mtx_img_);
  if (cache_img_.size() > cache_len_limit_) {
    CacheImgType img_msg = cache_img_.front();
    cache_img_.pop();
    auto drop_dnn_output = img_msg.first;
    std::string ts =
        std::to_string(drop_dnn_output->image_msg_header->stamp.sec) + "." +
        std::to_string(drop_dnn_output->image_msg_header->stamp.nanosec);
    RCLCPP_INFO(this->get_logger(),
                "drop cache_img_ ts %s",
                ts.c_str());
    // 可能只有图像消息，没有对应的AI消息
    if (drop_dnn_output->ai_msg) {
      msg_publisher_->publish(std::move(drop_dnn_output->ai_msg));
    }
  }
  CacheImgType cache_img = std::make_pair<std::shared_ptr<FeatOutput>,
                                          std::shared_ptr<NV12PyramidInput>>(
      std::move(dnn_output), std::move(pyramid));
  cache_img_.push(cache_img);
  cv_img_.notify_one();
  lg.unlock();
}

#ifdef SHARED_MEM_ENABLED
void FaceidNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  // dump recved img msg
  // std::ofstream ofs("img_" + std::to_string(img_msg->index) + "." +
  // std::string(reinterpret_cast<const char*>(img_msg->encoding.data())));
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data_size);

  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step << ", index: " << img_msg->index
     << ", stamp: " << img_msg->time_stamp.sec << "_"
     << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<NV12PyramidInput> pyramid = nullptr;
  if ("nv12" ==
      std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        img_msg->height,
        img_msg->width);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Unsupported img encoding: %s",
                img_msg->encoding);
  }
  if (!pyramid) {
    RCLCPP_ERROR(this->get_logger(), "Get Nv12 pym fail!");
    return;
  }

  // 2. 创建推理输出数据
  auto dnn_output = std::make_shared<FeatOutput>();
  // 将图片消息的header填充到输出数据中，用于表示推理输出对应的输入信息
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
  // 将当前的时间戳填充到输出数据中，用于计算perf
  dnn_output->perf_preprocess.stamp_start.sec = time_start.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_start.tv_nsec;
  dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
  dnn_output->img_h = img_msg->height;
  dnn_output->img_w = img_msg->width;

  if (dump_render_img_) {
    dnn_output->pyramid = pyramid;
  }

  // 3. 将准备好的输入输出数据存进缓存
  std::unique_lock<std::mutex> lg(mtx_img_);
  if (cache_img_.size() > cache_len_limit_) {
    CacheImgType img_msg = cache_img_.front();
    cache_img_.pop();
    auto drop_dnn_output = img_msg.first;
    std::string ts =
        std::to_string(drop_dnn_output->image_msg_header->stamp.sec) + "." +
        std::to_string(drop_dnn_output->image_msg_header->stamp.nanosec);
    RCLCPP_INFO(this->get_logger(),
                "drop cache_img_ ts %s",
                ts.c_str());
    // 可能只有图像消息，没有对应的AI消息
    if (drop_dnn_output->ai_msg) {
      msg_publisher_->publish(std::move(drop_dnn_output->ai_msg));
    }
  }
  CacheImgType cache_img = std::make_pair<std::shared_ptr<FeatOutput>,
                                          std::shared_ptr<NV12PyramidInput>>(
      std::move(dnn_output), std::move(pyramid));
  cache_img_.push(cache_img);
  cv_img_.notify_one();
  lg.unlock();
}
#endif

int FaceidNode::FeedFromLocal() {
  if (access(fb_img_info_.image_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(this->get_logger(),
                 "Image: %s not exist!",
                 fb_img_info_.image_.c_str());
    return -1;
  }

  std::ifstream ifs(fb_img_info_.image_, std::ios::in | std::ios::binary);
  if (!ifs) {
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  int len = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char* data = new char[len];
  ifs.read(data, len);

  // cv::Mat tmp;
  // cv::Mat mat(fb_img_info_.img_h * 3 / 2, fb_img_info_.img_w, CV_8UC1, data);
  // cv::cvtColor(mat, tmp, cv::COLOR_YUV2BGR_NV12);
  // cv::imwrite("origin_image.jpg", tmp);

  std::shared_ptr<NV12PyramidInput> pyramid = nullptr;
  pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(data),
      fb_img_info_.img_h,
      fb_img_info_.img_w,
      fb_img_info_.img_h,
      fb_img_info_.img_w);
  delete[] data;
  if (!pyramid) {
    RCLCPP_ERROR(this->get_logger(),
                 "Get Nv12 pym fail with image: %s",
                 fb_img_info_.image_.c_str());
    return -1;
  }

  auto rois = std::make_shared<std::vector<hbDNNRoi>>();
  for (int i = 0; i < fb_img_info_.rois.size(); i++) {
    hbDNNRoi roi;

    roi.left = fb_img_info_.rois[i][0];
    roi.top = fb_img_info_.rois[i][1];
    roi.right = fb_img_info_.rois[i][2];
    roi.bottom = fb_img_info_.rois[i][3];

    // roi's left and top must be even, right and bottom must be odd
    roi.left += (roi.left % 2 == 0 ? 0 : 1);
    roi.top += (roi.top % 2 == 0 ? 0 : 1);
    roi.right -= (roi.right % 2 == 1 ? 0 : 1);
    roi.bottom -= (roi.bottom % 2 == 1 ? 0 : 1);
    RCLCPP_DEBUG(this->get_logger(),
                "input face roi: %d %d %d %d",
                roi.left,
                roi.top,
                roi.right,
                roi.bottom);

    rois->push_back(roi);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  std::vector<std::shared_ptr<DNNInput>> inputs;
  for (size_t i = 0; i < rois->size(); i++) {
    inputs.push_back(pyramid);
  }
  auto dnn_output = std::make_shared<FeatOutput>();
  dnn_output->valid_rois = rois;
  dnn_output->valid_roi_idx[0] = 0;
  dnn_output->pyramid = pyramid;
  dnn_output->img_h = fb_img_info_.img_h;
  dnn_output->img_w = fb_img_info_.img_w;

  auto model_manage = GetModel();
  if (!model_manage) {
    RCLCPP_ERROR(this->get_logger(), "Invalid model");
    return -1;
  }

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    return -1;
  }
    
  return 0;
}

void FaceidNode::AiMsgProcess(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  if (!msg || !rclcpp::ok() || !ai_msg_manage_) {
    return;
  }

  std::stringstream ss;
  ss << "Recved ai msg"
     << ", frame_id: " << msg->header.frame_id
     << ", stamp: " << msg->header.stamp.sec << "_"
     << msg->header.stamp.nanosec << " target size: "
     << msg->targets.size();
  RCLCPP_INFO(
      this->get_logger(), "%s", ss.str().c_str());

  ai_msg_manage_->Feed(msg);
}

void FaceidNode::RunPredict() {
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lg(mtx_img_);
    cv_img_.wait(lg, [this]() { return !cache_img_.empty() || !rclcpp::ok(); });
    if (cache_img_.empty()) {
      continue;
    }
    if (!rclcpp::ok()) {
      break;
    }
    CacheImgType img_msg = cache_img_.front();
    cache_img_.pop();
    lg.unlock();

    auto dnn_output = img_msg.first;
    auto pyramid = img_msg.second;

    std::string ts =
        std::to_string(dnn_output->image_msg_header->stamp.sec) + "." +
        std::to_string(dnn_output->image_msg_header->stamp.nanosec);

    std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr;
    std::map<size_t, size_t> valid_roi_idx;
    ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg = nullptr;
    if (ai_msg_manage_->GetTargetRois(dnn_output->image_msg_header->stamp,
                                      rois,
                                      valid_roi_idx,
                                      ai_msg,
                                      std::bind(&FaceidNode::NormalizeRoi, this,
                                        std::placeholders::_1, std::placeholders::_2,
                                        expand_scale_, pyramid->width, pyramid->height),
                                      200) < 0 ||
        !ai_msg) {
      RCLCPP_INFO(this->get_logger(),
                  "Frame ts %s get face roi fail",
                  ts.c_str());
      continue;
    }
    if (!rois || rois->empty() || rois->size() != valid_roi_idx.size()) {
      RCLCPP_INFO(this->get_logger(),
                  "Frame ts %s has no face roi",
                  ts.c_str());
      if (!rois) {
        rois = std::make_shared<std::vector<hbDNNRoi>>();
      }
    }

    dnn_output->valid_rois = rois;
    dnn_output->valid_roi_idx = valid_roi_idx;
    dnn_output->ai_msg = std::move(ai_msg);

    auto model_manage = GetModel();
    if (!model_manage) {
      RCLCPP_ERROR(this->get_logger(), "Invalid model");
      continue;
    }

    // 2. 使用pyramid创建DNNInput对象inputs
    // inputs将会作为模型的输入通过RunInferTask接口传入
    std::vector<std::shared_ptr<DNNInput>> inputs;
    for (size_t i = 0; i < dnn_output->valid_rois->size(); i++) {
      inputs.push_back(pyramid);
    }

    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);
    dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
    dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;

    uint32_t ret = 0;
    // 3. 开始预测
    ret = Run(inputs, dnn_output, dnn_output->valid_rois, is_sync_mode_ == 1 ? true : false);
    // 4. 处理预测结果，如渲染到图片或者发布预测结果
    if (ret != 0) {
      continue;
    }
  }
}

int FaceidNode::NormalizeRoi(const hbDNNRoi *src,
                            hbDNNRoi *dst,
                            float norm_ratio,
                            uint32_t total_w,
                            uint32_t total_h) {
  *dst = *src;
  float box_w = dst->right - dst->left;
  float box_h = dst->bottom - dst->top;
  float center_x = (dst->left + dst->right) / 2.0f;
  float center_y = (dst->top + dst->bottom) / 2.0f;
  float w_new = box_w;
  float h_new = box_h;
  
  // {"norm_by_lside_ratio", NormMethod::BPU_MODEL_NORM_BY_LSIDE_RATIO},
  h_new = box_h * norm_ratio;
  w_new = box_w * norm_ratio;
  dst->left = center_x - w_new / 2;
  dst->right = center_x + w_new / 2;
  dst->top = center_y - h_new / 2;
  dst->bottom = center_y + h_new / 2;

  dst->left = dst->left < 0 ? 0.0f : dst->left;
  dst->top = dst->top < 0 ? 0.0f : dst->top;
  dst->right = dst->right > total_w ? total_w : dst->right;
  dst->bottom = dst->bottom > total_h ? total_h : dst->bottom;

  // roi's left and top must be even, right and bottom must be odd
  dst->left += (dst->left % 2 == 0 ? 0 : 1);
  dst->top += (dst->top % 2 == 0 ? 0 : 1);
  dst->right -= (dst->right % 2 == 1 ? 0 : 1);
  dst->bottom -= (dst->bottom % 2 == 1 ? 0 : 1);
 
  int32_t roi_w = dst->right - dst->left;
  int32_t roi_h = dst->bottom - dst->top;

  if (roi_w <= roi_w_size_max_ && roi_w >= roi_w_size_min_ &&
      roi_h <= roi_h_size_max_ && roi_h >= roi_h_size_min_ &&
      dst->left > 0 && dst->right < total_w) {
    // check success
    RCLCPP_DEBUG(this->get_logger(),
                  "Valid roi: %d %d %d %d, roi_w: %d, roi_h: %d.",
                  dst->left,
                  dst->top,
                  dst->right,
                  dst->bottom,
                  roi_w,
                  roi_h);
    return 0;
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "Filter roi: %d %d %d %d, roi_w: %d, roi_h: %d",
        dst->left,
        dst->top,
        dst->right,
        dst->bottom,
        roi_w,
        roi_h);
    if (roi_w > roi_w_size_max_ || roi_h > roi_h_size_max_) {
      RCLCPP_INFO(
          this->get_logger(),
          "Move far from sensor!");
    } else if (roi_w <= roi_w_size_min_ || roi_h > roi_h_size_min_) {
      RCLCPP_INFO(
          this->get_logger(),
          "Move close to sensor!");
    }

    return -1;
  }

  return 0;
}

