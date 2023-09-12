/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/


#include "play_football/play_football.h"


using hobot::dnn_node::DNNTensor;

namespace hobot {
namespace dnn_node {
namespace playfootball_node {
// 算法输出解析参数
struct PTQYolo5Config {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;
};

float score_threshold_ = 0.4;
float nms_threshold_ = 0.5;
int nms_top_k_ = 5000;

PTQYolo5Config yolo5_config_ = {
    {8, 16, 32},
    {{{10, 13}, {16, 30}, {33, 23}},
     {{30, 61}, {62, 45}, {59, 119}},
     {{116, 90}, {156, 198}, {373, 326}}},
    1,
    {"football"}
};

void ParseTensor(std::shared_ptr<DNNTensor> tensor,
                 int layer,
                 std::vector<YoloV5Result> &results) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  int num_classes = yolo5_config_.class_num;
  int stride = yolo5_config_.strides[layer];
  int num_pred = yolo5_config_.class_num + 4 + 1;

  std::vector<float> class_pred(yolo5_config_.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      yolo5_config_.anchors_table[layer];

  //  int *shape = tensor->data_shape.d;
  int height, width;
  auto ret = get_tensor_hw(tensor, &height, &width);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                 "get_tensor_hw failed");
  }

  int anchor_num = anchors.size();
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      for (int k = 0; k < anchor_num; k++) {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        float *cur_data = data + k * num_pred;
        float objness = cur_data[4];

        int id = argmax(cur_data + 5, cur_data + 5 + num_classes);
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-cur_data[id + 5]));
        double confidence = x1 * x2;

        if (confidence < score_threshold_) {
          continue;
        }

        float center_x = cur_data[0];
        float center_y = cur_data[1];
        float scale_x = cur_data[2];
        float scale_y = cur_data[3];

        double box_center_x =
            ((1.0 / (1.0 + std::exp(-center_x))) * 2 - 0.5 + w) * stride;
        double box_center_y =
            ((1.0 / (1.0 + std::exp(-center_y))) * 2 - 0.5 + h) * stride;

        double box_scale_x =
            std::pow((1.0 / (1.0 + std::exp(-scale_x))) * 2, 2) * anchor_x;
        double box_scale_y =
            std::pow((1.0 / (1.0 + std::exp(-scale_y))) * 2, 2) * anchor_y;

        double xmin = (box_center_x - box_scale_x / 2.0);
        double ymin = (box_center_y - box_scale_y / 2.0);
        double xmax = (box_center_x + box_scale_x / 2.0);
        double ymax = (box_center_y + box_scale_y / 2.0);

        if (xmax <= 0 || ymax <= 0) {
          continue;
        }

        if (xmin > xmax || ymin > ymax) {
          continue;
        }

        results.emplace_back(
            YoloV5Result(static_cast<int>(id),
                         xmin,
                         ymin,
                         xmax,
                         ymax,
                         confidence,
                         yolo5_config_.class_names[static_cast<int>(id)]));
      }
      data = data + num_pred * anchors.size();
    }
  }
}

void yolo5_nms(std::vector<YoloV5Result> &input,
               float iou_threshold,
               int top_k,
               std::vector<std::shared_ptr<YoloV5Result>> &result,
               bool suppress) {
  // sort order by score desc
  std::stable_sort(input.begin(), input.end(), std::greater<YoloV5Result>());

  std::vector<bool> skip(input.size(), false);

  // pre-calculate boxes area
  std::vector<float> areas;
  areas.reserve(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float width = input[i].xmax - input[i].xmin;
    float height = input[i].ymax - input[i].ymin;
    areas.push_back(width * height);
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (input[i].id != input[j].id) {
          continue;
        }
      }

      // intersection area
      float xx1 = std::max(input[i].xmin, input[j].xmin);
      float yy1 = std::max(input[i].ymin, input[j].ymin);
      float xx2 = std::min(input[i].xmax, input[j].xmax);
      float yy2 = std::min(input[i].ymax, input[j].ymax);

      if (xx2 > xx1 && yy2 > yy1) {
        float area_intersection = (xx2 - xx1) * (yy2 - yy1);
        float iou_ratio =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou_ratio > iou_threshold) {
          skip[j] = true;
        }
      }
    }

    auto yolo_res = std::make_shared<YoloV5Result>(input[i].id,
                                                   input[i].xmin,
                                                   input[i].ymin,
                                                   input[i].xmax,
                                                   input[i].ymax,
                                                   input[i].score,
                                                   input[i].class_name);
    if (!yolo_res) {
      RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                   "invalid yolo_res");
    }

    result.push_back(yolo_res);
  }
}

int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width) {
  int h_index = 0;
  int w_index = 0;
  if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    h_index = 1;
    w_index = 2;
  } else if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    h_index = 2;
    w_index = 3;
  } else {
    return -1;
  }
  *height = tensor->properties.validShape.dimensionSize[h_index];
  *width = tensor->properties.validShape.dimensionSize[w_index];
  return 0;
}

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::vector<std::shared_ptr<YoloV5Result>> &results) {
  std::vector<YoloV5Result> parse_results;
  for (size_t i = 0; i < node_output->output_tensors.size(); i++) {
    ParseTensor(
        node_output->output_tensors[i], static_cast<int>(i), parse_results);
  }

  yolo5_nms(parse_results, nms_threshold_, nms_top_k_, results, false);

  return 0;
}

} 
} 
}  


// 使用hobotcv resize nv12格式图片，固定图片宽高比
int ResizeNV12Img(const char* in_img_data,
                  const int& in_img_height,
                  const int& in_img_width,
                  const int& scaled_img_height,
                  const int& scaled_img_width,
                  cv::Mat& out_img,
                  float& ratio) {
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void*)(in_img_data));
  float ratio_w =
      static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h =
      static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w) {
    resized_width = scaled_img_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  } else if (dst_ratio == ratio_h) {
    resized_width = static_cast<float>(in_img_width) / dst_ratio;
    resized_height = scaled_img_height;
  }

  // hobot_cv要求输出宽度为16的倍数
  int remain = resized_width % 16;
  if (remain != 0) {
    //向下取16倍数，重新计算缩放系数
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  //高度向下取偶数
  resized_height =
      resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

struct Football_output : public hobot::dnn_node::DnnNodeOutput {
  // 缩放比例系数，原图和模型输入分辨率的比例。
  float ratio = 1.0;
};



Football_node::Football_node(const std::string& node_name,
                             const rclcpp::NodeOptions& options)
    : hobot::dnn_node::DnnNode(node_name, options) {
  if (Init() != 0 ||
      GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"), "Node init fail!");
    rclcpp::shutdown();
  }
  ros_img_subscription_ =
      this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
          "/hbmem_img",
          10,
          std::bind(&Football_node::FeedImg, this, std::placeholders::_1));
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      "/playfootball_node", 10);
  cmd_vel_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
}

int Football_node::SetNodePara() {
  if (!dnn_node_para_ptr_) return -1;
  dnn_node_para_ptr_->model_file =
      "/userdata/dev_ws/src/originbot/originbot_deeplearning/play_football/config/play_football.bin";
  dnn_node_para_ptr_->model_task_type =
      hobot::dnn_node::ModelTaskType::ModelInferType;
  dnn_node_para_ptr_->task_num = 4;
  return 0;
}

void Football_node::FeedImg(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!rclcpp::ok() || !img_msg) {
    return;
  }

  // 1 对订阅到的图片消息进行验证
  if ("nv12" !=
      std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),
                 "Please use nv12 img encoding!.");
    return;
  }

  // 2 创建算法输出数据，填充消息头信息，用于推理完成后AI结果的发布
  auto dnn_output = std::make_shared<Football_output>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->msg_header->set__stamp(img_msg->time_stamp);

  // 3 算法前处理，即创建算法输入数据
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
      img_msg->width != static_cast<uint32_t>(model_input_width_)) {
    // 3.1 订阅到的图片和算法输入分辨率不一致，需要做resize处理
    cv::Mat out_img;
    if (ResizeNV12Img(reinterpret_cast<const char*>(img_msg->data.data()),
                      img_msg->height,
                      img_msg->width,
                      model_input_height_,
                      model_input_width_,
                      out_img,
                      dnn_output->ratio) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),
                   "Resize nv12 img fail!");
      return;
    }

    uint32_t out_img_width = out_img.cols;
    uint32_t out_img_height = out_img.rows * 2 / 3;
    // 3.2 根据算法输入图片分辨率，使用hobot_dnn中提供的方法创建算法输入数据
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(out_img.data),
        out_img_height,
        out_img_width,
        model_input_height_,
        model_input_width_);
  } else {
    // 3.3不需要进行resize，直接根据算法输入图片分辨率，使用hobot_dnn中提供的方法创建算法输入数据
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  }
  // 3.4 校验算法输入数据
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"), "Get pym fail");
    return;
  }
  // 3.5 将算法输入数据转成dnn node推理输入的格式
  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};


  // 4 使用创建的算法输入和输出数据，以异步模式运行推理，推理结果通过PostProcess接口回调返回
  if (Run(inputs, dnn_output, nullptr, false) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Run predict fail!");
  }
}

int Football_node::PostProcess(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  auto tp_start = std::chrono::system_clock::now();

  // 1 创建用于发布推理结果的ROS Msg
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());

  float target_point=0;
  int exist_result=0;
  // 2 将推理输出对应图片的消息头填充到ROS Msg
  pub_data->set__header(*node_output->msg_header);


  // 3创建解析输出数据，输出YoloV5Result是自定义的算法输出数据类型，results的维度等于检测出来的目标数
  std::vector<std::shared_ptr<hobot::dnn_node::playfootball_node::YoloV5Result>>
      results;

  if (hobot::dnn_node::playfootball_node::Parse(node_output, results) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),
                 "Parse node_output fail!");
    return -1;
  }

  for (auto& rect : results) {
    if (!rect) {exist_result=0;continue;}
    
    if (rect->xmin < 0) rect->xmin = 0;
    if (rect->ymin < 0) rect->ymin = 0;
    if (rect->xmax >= model_input_width_) {
      rect->xmax = model_input_width_ - 1;
    }
    if (rect->ymax >= model_input_height_) {
      rect->ymax = model_input_height_ - 1;
    }
    if(rect->score >= 0.65) exist_result=1;

    std::stringstream ss;
    ss << "roi rect: " << rect->xmin << " " << rect->ymin << " " << rect->xmax
       << " " << rect->ymax << ", roi type: " << rect->class_name
       << ", score:" << rect->score;
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(rect->xmin);
    roi.rect.set__y_offset(rect->ymin);
    roi.rect.set__width(rect->xmax - rect->xmin);
    roi.rect.set__height(rect->ymax - rect->ymin);
    roi.set__confidence(rect->score);

    ai_msgs::msg::Target target;
    target.set__type(rect->class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));

  }

  // 4 坐标映射
  auto sample_node_output =
      std::dynamic_pointer_cast<Football_output>(node_output);
  if (!sample_node_output) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),
                 "Cast dnn node output fail!");
    return -1;
  }
  if (sample_node_output->ratio != 1.0) {
    for (auto& target : pub_data->targets) {
      for (auto& roi : target.rois) {
        roi.rect.x_offset *= sample_node_output->ratio;
        roi.rect.y_offset *= sample_node_output->ratio;
        roi.rect.width *= sample_node_output->ratio;
        roi.rect.height *= sample_node_output->ratio;
        
        target_point = roi.rect.x_offset + 0.5 * roi.rect.width;
      }
    }
  }


  // 5 机器运动控制
  auto message = geometry_msgs::msg::Twist();
  
  if(exist_result == 1){
      message.angular.z = -1.0 * (target_point - 480) / 300.0; 
      message.linear.x  = 0.24;
  }
  else {
      message.angular.z = 0.15;
      message.linear.x  = 0;
  }
  message.linear.y = 0.0;
  message.linear.z = 0.0;
  message.angular.x = 0.0;
  message.angular.y = 0.0;
  cmd_vel_publisher_->publish(message);

  // 6 将算法推理输出帧率填充到ROS Msg
  if (node_output->rt_stat) {
    pub_data->set__fps(round(node_output->rt_stat->output_fps));
    // 如果算法推理统计有更新，输出算法输入和输出的帧率统计、推理耗时
    if (node_output->rt_stat->fps_updated) {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_INFO(rclcpp::get_logger("playfootball_node"),
                  "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                  "post process time ms: %d",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  node_output->rt_stat->infer_time_ms,
                  interval);
    }
  }

  // 7 发布ROS Msg
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Football_node>());
  rclcpp::shutdown();
  return 0;
}
