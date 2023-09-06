//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <string>
#include <tuple>

#include "common/configurator.h"

namespace Scripts {

struct Uart {
    bool enable;

    std::string name;
    int baudrate;
};

struct Ethernet {
    bool enable;

    std::string radar_ip;
    int radar_port;

    int listen_port;
};

// 通信协议相关配置
struct Protocol {
    Uart uart;
    Ethernet ethernet;
};

bool GetProtocol(std::unique_ptr<Configurator> configurator, Protocol &protocol);

// 离群点滤波器参数
struct OFParams {
    bool enable;

    double radius;
    int min_count;
    double max_distance;
};

struct OFNAParams {
    double front_angle;
    double side_angle;
    double lidar_angle;
    bool angle;
};

// 平滑滤波器参数
struct SFParams {
    bool enable;

    int level;
    double err;
};

// 拖尾滤波器参数
struct TFParams {
    bool enable;
    int window;
    int neighbors;
    double min_angle;
    double max_angle;
};

// 角度滤波器参数
struct AFParams {
    bool enable;
    std::vector<std::tuple<double, double>> removed;
};

struct Filters {
    OFParams outlier_filter;
    OFParams outlier_filter_near;
    OFNAParams outlier_filter_near_angle;
    SFParams smooth_filter;
    TFParams trailing_filter;
    AFParams angle_filter;
};

bool GetFiltersOptions(std::unique_ptr<Configurator> configurator, Filters &filters);

// 位姿相关配置
struct Pose {
    double rotate_angle;
};

std::vector<std::tuple<double, double>> GetRangeWithPose(const std::vector<std::tuple<double, double>> &range, double rotate);

// 雷达属性定义
struct LidarAttribute {
    double max_range;
    double zero_angle;
};

bool GetLidarAttr(std::unique_ptr<Configurator> configurator, LidarAttribute &attribute);
}