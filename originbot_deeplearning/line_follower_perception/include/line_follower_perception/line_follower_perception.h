#ifndef _LINE_FOLLOWER_PERCEPTION_H_
#define _LINE_FOLLOWER_PERCEPTION_H_

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/dnn_node_data.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::ModelTaskType;

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::OutputDescription;
using hobot::dnn_node::OutputParser;

using hobot::dnn_node::InputDescription;
using hobot::dnn_node::SingleBranchOutputParser;

class LineCoordinateResult : public DNNResult {
 public:
  float x;
  float y;
  void Reset() override {x = -1.0; y = -1.0;}
};

class LineCoordinateParser
  : public SingleBranchOutputParser<LineCoordinateResult> {
 public:
  LineCoordinateParser() {}
  ~LineCoordinateParser() {}
  int32_t Parse(
      std::shared_ptr<LineCoordinateResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor) override;
};

class LineFollowerPerceptionNode : public DnnNode {
 public:
  LineFollowerPerceptionNode(const std::string& node_name,
                        const NodeOptions &options = NodeOptions());
  ~LineFollowerPerceptionNode() override;

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  int Predict(std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
              const std::shared_ptr<DnnNodeOutput> &output,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois);
  void subscription_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg);
  bool GetParams();
  bool AssignParams(const std::vector<rclcpp::Parameter> & parameters);
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
    subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  cv::Mat image_bgr_;
  std::string model_path_;
  std::string model_name_;
};

#endif  // _LINE_FOLLOWER_PERCEPTION_H_