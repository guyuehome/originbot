-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                    -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,      -- trajectory_builder.lua的配置信息
  map_frame = "map",                            -- 地图坐标系的名称
  tracking_frame = "base_footprint",            -- 跟踪坐标系，一般是机器人基坐标系的名称
  published_frame = "odom",                     -- 发布定位信息所使用的里程计坐标系名称，算法将发布map和published_frame之间的tf
  odom_frame = "odom",                          -- 机器人现有里程计坐标系的名称
  provide_odom_frame = false,                   -- 是否需要算法发布里程计信息
  publish_frame_projected_to_2d = false,        -- 是否只发布二维姿态信息
  use_odometry = true,                          -- 是否使用机器人里程计，如果是，机器人必须要发布odom的tf
  use_nav_sat = false,                          -- 是否使用GPS
  use_landmarks = false,                        -- 是否使用路标
  num_laser_scans = 1,                          -- 订阅激光雷达LaserScan话题的数量
  num_multi_echo_laser_scans = 0,               -- 订阅多回波技术激光雷达话题的数量
  num_subdivisions_per_laser_scan = 1,          -- 将一帧激光雷达数据分割为几次处理
  num_point_clouds = 0,                         -- 是否使用点云数据
  lookup_transform_timeout_sec = 0.2,           -- 查找tf坐标变换数据的超时时间 
  submap_publish_period_sec = 0.3,              -- 发布submap子图的周期，单位秒
  pose_publish_period_sec = 5e-3,               -- 发布姿态的周期，单位秒
  trajectory_publish_period_sec = 30e-3,        -- 发布轨迹的周期，单位秒
  rangefinder_sampling_ratio = 1.,              -- 测距仪的采样频率
  odometry_sampling_ratio = 1.,                 -- 里程计数据采样率
  fixed_frame_pose_sampling_ratio = 1.,         -- 固定坐标系位姿采样率
  imu_sampling_ratio = 1.,                      -- IMU数据采样率
  landmarks_sampling_ratio = 1.,                -- 路标数据采样率
}

MAP_BUILDER.use_trajectory_builder_2d = true          -- 是否使用2D建图                

TRAJECTORY_BUILDER_2D.min_range = 0.1                 -- 激光雷达监测的最小距离，单位米
TRAJECTORY_BUILDER_2D.max_range = 8                   -- 激光雷达监测的最大距离，单位米
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.5   -- 无效激光数据设置为该数值，滤波的时候使用
TRAJECTORY_BUILDER_2D.use_imu_data = false            -- 是否使用IMU的数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true      -- 是否使用CSM激光匹配
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 两帧激光雷达数据的最小角度

POSE_GRAPH.constraint_builder.min_score = 0.7                          -- 全局约束当前最小得分(当前node与当前submap的匹配得分)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7      -- 全局约束全局最小得分(当前node与全局submap的匹配得分)

--POSE_GRAPH.optimize_every_n_nodes = 30                               -- 每次整体优化间隔nodes数

return options
