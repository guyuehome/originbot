// Copyright (c) 2022，Horizon Robotics.
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

#include "include/parking_search_engine.h"

#include <fstream>
#include <memory>
#include <sstream>
#include <vector>
#include <utility>

std::shared_ptr<ParkingSearchEngine> ParkingSearchEngine::Instance() {
  static std::shared_ptr<ParkingSearchEngine> inst =
      std::shared_ptr<ParkingSearchEngine>(new ParkingSearchEngine());
  return inst;
}

ParkingSearchEngine::ParkingSearchEngine() {
  RCLCPP_INFO(rclcpp::get_logger("ParkingSearchEngine"),
              "ParkingSearchEngine construct");
  start_ = true;

  param_node_ = std::make_shared<ParametersClass>(&cfg_);

  parking_search_node_ = std::make_shared<ParkingSearchNode>(
      "parking_search",
      std::bind(&ParkingSearchEngine::FeedSmart, this, std::placeholders::_1));

  if (!smart_process_task_) {
    smart_process_task_ = std::make_shared<std::thread>([this]() {
      while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(smart_queue_mtx_);
        smart_queue_condition_.wait_for(lg, std::chrono::seconds(1), [&]() {
          return !smart_queue_.empty();
        });
        if (smart_queue_.empty() || !rclcpp::ok()) {
          continue;
        }
        auto smart_frame = std::move(smart_queue_.front());
        smart_queue_.pop();
        lg.unlock();
        RunStrategy(smart_frame);
      }

      // 退出前发布停止运动指令，避免程序退出后机器人还一直处于运动状态（如果最后一次收到的指令是启动运动并且运动控制模块没有做超时管理）
      RCLCPP_WARN(rclcpp::get_logger("ParkingSearchEngine"),
                  "pkg exit! cancel move");
      CancelMove();
    });
  }
}

ParkingSearchEngine::~ParkingSearchEngine() {
  RCLCPP_INFO(rclcpp::get_logger("ParkingSearchEngine"),
              "ParkingSearchEngine deconstruct");
  start_ = false;

  if (smart_process_task_ && smart_process_task_->joinable()) {
    smart_process_task_->join();
    smart_process_task_ = nullptr;
  }
}

void ParkingSearchEngine::DoRotate(int direction, float step) {
  last_ctrl_is_cancel_ = false;
  RCLCPP_WARN(rclcpp::get_logger("ParkingSearchEngine"),
              "do rotate, direction: %d, step: %f",
              direction,
              step);

  int direct = 1;
  if (static_cast<int>(DirectionType::RIGHT) == direction) {
    direct = -1;
  }

  auto twist = std::make_shared<Twist>();
  twist->angular.z = direct * step;
  FeedMovePoseMsg(twist);

  RCLCPP_INFO(rclcpp::get_logger("ParkingSearchEngine"),
              "present frame_ts_ms: %llu",
              search_info_.frame_ts_ms);
}

void ParkingSearchEngine::DoMove(int direction, float step) {
  last_ctrl_is_cancel_ = false;
  RCLCPP_WARN(rclcpp::get_logger("ParkingSearchEngine"),
              "do move, direction: %d, step: %f",
              direction,
              step);
  auto twist = std::make_shared<Twist>();
  if (0 == direction) {
    twist->linear.x += step;
  } else if (1 == direction) {
    twist->linear.x -= step;
  } else if (2 == direction) {
    twist->linear.y += step;
  } else if (3 == direction) {
    twist->linear.y -= step;
  }

  FeedMovePoseMsg(twist);
}

void ParkingSearchEngine::CancelMove() {
  if (last_ctrl_is_cancel_) return;
  last_ctrl_is_cancel_ = true;
  RCLCPP_WARN(rclcpp::get_logger("ParkingSearchEngine"), "cancel move");
  auto twist = std::make_shared<Twist>();
  twist->linear.x = 0;
  twist->linear.y = 0;
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = 0;
  FeedMovePoseMsg(twist);
}

void ParkingSearchEngine::FeedMovePoseMsg(const Twist::SharedPtr &pose) {
  if (parking_search_node_ && pose) {
    parking_search_node_->RobotCtl(*pose);
  }
}

void ParkingSearchEngine::ProcessSmart(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &ai_msg) {
  // update search info

  if(search_info_.finalcount > cfg_.arrived_count){
    search_info_.search_sta = SearchStatus::ARRIVED;
    return;
  }

  if(search_info_.find_parking){
    RCLCPP_WARN(rclcpp::get_logger("ParkingSearchEngine"), 
                "Find Target, current count: %d, target count: %d", 
                search_info_.finalcount,
                cfg_.arrived_count);
  }
  

  if (!ai_msg->targets.empty()) {
    search_info_.search_sta = SearchStatus::SEARCHING;
  }else{
    search_info_.search_sta = SearchStatus::INITING;
    RCLCPP_DEBUG(rclcpp::get_logger("ParkingSearchEngine"),
            "No ai_msg received!");
  }
        
  if (SearchStatus::SEARCHING == search_info_.search_sta) {
    int goal = GetStrategy(ai_msg);

    // 更新当前帧的信息
    search_info_.frame_ts_ms = ai_msg->header.stamp.sec * 1000 +
                              ai_msg->header.stamp.nanosec / 1000 / 1000;
    search_info_.goal = goal;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ParkingSearchEngine"),
              "search_sta: %d (0:INITING, 1:SEARCHING, 2:ARRIVED), "
              "goal: %d, frame_ts_ms: %llu",
              search_info_.search_sta,
              search_info_.goal,
              search_info_.frame_ts_ms);
}

int ParkingSearchEngine::GetStrategy(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &ai_msg){

    for (const auto target : ai_msg->targets) {
      RCLCPP_INFO(
          rclcpp::get_logger("ParkingSearchEngine"),
          "target.rois.size: %d, target.capture.size: %d",
          target.rois.size(),
          target.captures.size());

      int lock_index = cfg_.img_mid_width;
      for (const auto &roi : target.rois) {
        RCLCPP_DEBUG(rclcpp::get_logger("ParkingSearchEngine"),
                     "roi.type: %s",
                     roi.type.c_str());

        if ("parking_lock" != roi.type) {
          continue;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("ParkingSearchEngine"),
                     "vehicle roi x_offset: %d y_offset: %d width: %d height: %d",
                     roi.rect.x_offset,
                     roi.rect.y_offset,
                     roi.rect.width,
                     roi.rect.height);

        lock_index = static_cast<int>((lock_index + roi.rect.x_offset + static_cast<int>(roi.rect.width / 2) / 2));
      }
      if(lock_index > cfg_.img_mid_width){
        return static_cast<int>(CtrlType::GoRight);
      }else if(lock_index < cfg_.img_mid_width){
        return static_cast<int>(CtrlType::GoLeft);
      }
      
      for (const auto& capture : target.captures) {

        int width = capture.img.width;
        int height = capture.img.height;
        auto feature = capture.features;

        std::vector<int> path_space = {0, 0, 0};
        std::vector<int> parking_space = {0, 0, 0};
        std::vector<float> parking_ious = {cfg_.sides_parking_iou, 
                                          cfg_.mid_parking_iou,
                                          cfg_.sides_parking_iou};
        std::vector<float> path_ious = {cfg_.sides_path_iou, 
                                        cfg_.mid_path_iou,
                                        cfg_.sides_path_iou};

        int width_start = int((width - 2 * cfg_.area_width) / 2);
        
        for(int h = height - cfg_.ingored_bottom - cfg_.area_height; h < height - cfg_.ingored_bottom; h++){
          int index = h * width + width_start;
          for(int i = 0; i < 3; i++){
            for(int w = i * int(cfg_.area_width / 2); w < i * int(cfg_.area_width / 2) + cfg_.area_width; w++){
              if(feature[index + w] == 0 || feature[index + w] == 2 || feature[index + w] == 3){
                path_space[i]++;
              }else if(feature[index + w] == 4 || feature[index + w] == 5){
                parking_space[i]++;
              }
            }
          }
        }

        int areas = cfg_.area_height * cfg_.area_width;
        std::vector<int> parking_areas = {0, 0, 0};
        std::vector<int> path_areas = {0, 0, 0};
        std::vector<int> status = {0, 0, 0};

        for(int i = 0; i < 3; i++){
          parking_areas[i] = static_cast<int>(parking_ious[i] * areas);
          path_areas[i] = static_cast<int>(path_ious[i] * areas);
          if((path_space[i] + parking_space[i]) > path_areas[i]){
            status[i] = 1;
          }
          if(parking_space[i] > parking_areas[i]){
            status[i] = 2;
          }
        }

        if(status[0] == 2 && status[1] == 2 && status[2] == 2){
          search_info_.find_parking = true;
          search_info_.finalcount++;
          return static_cast<int>(CtrlType::GoFront);
        }

        if(search_info_.finalcount > 0){
          search_info_.finalcount--;
        }
        
        if(status[1] == 2){
          if(parking_space[0] > parking_space[1]){
            return static_cast<int>(CtrlType::GoLeft);
          }else if(parking_space[2] > parking_space[1]){
            return static_cast<int>(CtrlType::GoRight);
          }
          return static_cast<int>(CtrlType::GoFront);
        }else if(status[0] == 2){
          return static_cast<int>(CtrlType::GoLeft);
        }else if(status[2] == 2){
          return static_cast<int>(CtrlType::GoRight);
        }

        if(status[1] == 1){
          return static_cast<int>(CtrlType::GoFront);
        }else if(status[0] == 1){
          return static_cast<int>(CtrlType::GoLeft);
        }else if(status[2] == 1){
          return static_cast<int>(CtrlType::GoRight);
        }else{
          search_info_.find_parking = false;
          search_info_.finalcount = 0;
          return static_cast<int>(CtrlType::GoBack);
        }
      }
    }
    return static_cast<int>(CtrlType::Stay);
}

void ParkingSearchEngine::RunStrategy(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  if (!msg || !rclcpp::ok()) {
    return;
  }

  ProcessSmart(msg);

  if (SearchStatus::INITING == search_info_.search_sta) {
    RCLCPP_DEBUG(rclcpp::get_logger("ParkingSearchEngine"), "search is initing");
    CancelMove();
    return;
  }else if (SearchStatus::ARRIVED == search_info_.search_sta) {
    RCLCPP_WARN(rclcpp::get_logger("ParkingSearchEngine"), "Parking Area Arrived !!!");
    CancelMove();
    return;
  }

  if (static_cast<int>(CtrlType::GoFront) != search_info_.goal &&
      static_cast<int>(CtrlType::GoBack) != search_info_.goal &&
      static_cast<int>(CtrlType::GoRight) != search_info_.goal &&
      static_cast<int>(CtrlType::GoLeft) != search_info_.goal) {
    CancelMove();
    return;
  }

  // RCLCPP_WARN(rclcpp::get_logger("ParkingSearchEngine"),
  //             "frame_ts_ms: %llu, search_sta: %d, goal: %d",
  //             search_info_.frame_ts_ms,
  //             static_cast<int>(search_info_.search_sta),
  //             static_cast<int>(search_info_.goal));

  if (static_cast<int>(CtrlType::GoFront) == 
            search_info_.goal) {
    // move front
    DoMove(static_cast<int>(DirectionType::FRONT), cfg_.move_step);
    return;
  } else if (static_cast<int>(CtrlType::GoBack) ==
             search_info_.goal) {
    // move back
    DoMove(static_cast<int>(DirectionType::BACK), cfg_.move_step);
    return;
  } else if (static_cast<int>(CtrlType::GoRight) ==
             search_info_.goal) {
    // rotate right
    DoRotate(static_cast<int>(DirectionType::RIGHT),
             cfg_.rotate_step);
    return;
  } else if (static_cast<int>(CtrlType::GoLeft) ==
             search_info_.goal) {
    // rotate left
    DoRotate(static_cast<int>(DirectionType::LEFT),
             cfg_.rotate_step);
    return;
  }

  CancelMove();
  return;
}

void ParkingSearchEngine::FeedSmart(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  std::unique_lock<std::mutex> lg(smart_queue_mtx_);
  smart_queue_.push(msg);
  if (smart_queue_.size() > queue_len_limit_) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkingSearchEngine"),
                 "smart queue len exceed limit: %d",
                 queue_len_limit_);
    smart_queue_.pop();
  }
  smart_queue_condition_.notify_one();
  lg.unlock();
}
