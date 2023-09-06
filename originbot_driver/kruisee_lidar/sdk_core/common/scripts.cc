//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "common/scripts.h"

#include <string.h>

#include <iostream>

namespace Scripts {

static bool IsIPv4(const std::string &ip)
{
    int count = 0;
    int begin = 0, end = 0;
    while (end < ip.length()) {
        if ((ip.at(end) != '.') && ((ip.at(end) < '0') || (ip.at(end) > '9')))
            return false;

        if (ip.at(end) != '.') {
            end++;
            continue;
        }

        int num = atoi(ip.substr(begin, end - begin).c_str());
        if (num < 0 || num > 255)
            return false;

        count++;

        end += 1;
        begin = end;

        if (end == ip.length())
            break;
    }

    if (begin == end)
        return false;

    int num = atoi(ip.substr(begin, ip.length() - begin).c_str());
    if (num < 0 || num > 255)
        return false;

    count++;
    if (count == 4)
        return true;

    return false;
}

bool GetProtocol(std::unique_ptr<Configurator> configurator, Protocol &protocol)
{
    if (!configurator->GetBool("use_net_protocol", protocol.ethernet.enable))
        return false;

    auto ethernet = configurator->GetExtractor("ethernet");
    if (ethernet == nullptr)
        return false;

    if (!ethernet->GetString("radar_ip", protocol.ethernet.radar_ip))
        return false;

    if (!IsIPv4(protocol.ethernet.radar_ip)) {
        std::cerr << "Illegal IP address" << std::endl;
        return false;
    }

    if (!ethernet->GetInt("radar_port", protocol.ethernet.radar_port))
        return false;

    if (protocol.ethernet.radar_port < 0 || protocol.ethernet.radar_port > 65535) {
        std::cerr << "Illegal radar port parameters" << std::endl;
        return false;
    }

    if (!ethernet->GetInt("listen_port", protocol.ethernet.listen_port))
        return false;

    if (protocol.ethernet.listen_port < 0 || protocol.ethernet.listen_port > 65535) {
        std::cerr << "Illegal PC listening port parameters" << std::endl;
        return false;
    }

    if(!configurator->GetBool("use_uart_protocol", protocol.uart.enable))
        return false;

    auto uart = configurator->GetExtractor("uart");
    if (uart == nullptr)
        return false;

    if (!uart->GetString("name", protocol.uart.name))
        return false;

    if (!uart->GetInt("baudrate", protocol.uart.baudrate))
        return false;

    if (protocol.uart.baudrate < 0) {
        std::cerr << "Illegal uart baudrate" << std::endl;
        return false;
    }

    return true;
}

bool GetFiltersOptions(std::unique_ptr<Configurator> configurator, Filters &filters)
{
    if (!configurator->GetBool("use_outlier_filter", filters.outlier_filter.enable))
        return false;

    auto rf = configurator->GetExtractor("outlier_filter");
    if (rf == nullptr)
        return false;

    if (!rf->GetDouble("radius", filters.outlier_filter.radius))
        return false;

    if (filters.outlier_filter.radius < 0) {
        std::cerr << "Illegal filter radius" << std::endl;
        return false;
    }

    if (!rf->GetInt("min_count", filters.outlier_filter.min_count))
        return false;

    if (filters.outlier_filter.min_count < 0) {
        std::cerr << "Illegal minimum number of adjacent point sets" << std::endl;
        return false;
    }

    if (!rf->GetDouble("max_distance", filters.outlier_filter.max_distance))
        return false;

    if (filters.outlier_filter.max_distance < 0) {
        std::cerr << "Illegal maximum filter distance" << std::endl;
        return false;
    }


    if (!configurator->GetBool("use_outlier_filter_near", filters.outlier_filter_near.enable))
        return false;

    auto rfn = configurator->GetExtractor("outlier_filter_near");
    if (rfn == nullptr)
        return false;

    if (!rfn->GetDouble("radius", filters.outlier_filter_near.radius))
        return false;

    if (filters.outlier_filter_near.radius < 0) {
        std::cerr << "Illegal filter radius" << std::endl;
        return false;
    }

    if (!rfn->GetInt("min_count", filters.outlier_filter_near.min_count))
        return false;

    if (filters.outlier_filter_near.min_count < 0) {
        std::cerr << "Illegal minimum number of adjacent point sets" << std::endl;
        return false;
    }

    if (!rfn->GetDouble("max_distance", filters.outlier_filter_near.max_distance))
        return false;

    if (filters.outlier_filter_near.max_distance < 0) {
        std::cerr << "Illegal maximum filter distance" << std::endl;
        return false;
    }

    if (!rfn->GetDouble("front_angle", filters.outlier_filter_near_angle.front_angle))
        return false;

    if (!rfn->GetDouble("side_angle", filters.outlier_filter_near_angle.side_angle))
        return false;

    if (!rfn->GetDouble("lidar_angle", filters.outlier_filter_near_angle.lidar_angle))
        return false;

    if (!rfn->GetBool("angle", filters.outlier_filter_near_angle.angle))
        return false;




    if (!configurator->GetBool("use_smooth_filter", filters.smooth_filter.enable))
        return false;

    auto sf = configurator->GetExtractor("smooth_filter");
    if (sf == nullptr)
        return false;

    if (!sf->GetInt("level", filters.smooth_filter.level))
        return false;

    if (filters.smooth_filter.level < 0 || filters.smooth_filter.level > 50) {
        std::cerr << "Illegal smooth filter level" << std::endl;
        return false;
    }

    if (!sf->GetDouble("err", filters.smooth_filter.err))
        return false;

    if (filters.smooth_filter.err < 0) {
        std::cerr << "Illegal smooth filter err" << std::endl;
        return false;
    }

    if (!configurator->GetBool("use_trailing_filter", filters.trailing_filter.enable))
        return false;

    auto tf = configurator->GetExtractor("trailing_filter");
    if (tf == nullptr)
        return false;

    if (!tf->GetInt("window", filters.trailing_filter.window))
        return false;

    if (filters.trailing_filter.window < 0) {
        std::cerr << "Illegal trailing filter window" << std::endl;
        return false;
    }

    if (!tf->GetInt("neighbors", filters.trailing_filter.neighbors))
        return false;

    if (filters.trailing_filter.neighbors < 0) {
        std::cerr << "Illegal trailing filter neighbors" << std::endl;
        return false;
    }

    if (!tf->GetDouble("min_angle", filters.trailing_filter.min_angle))
        return false;

    if (filters.trailing_filter.min_angle < 0 || filters.trailing_filter.min_angle > 360) {
        std::cerr << "Illegal trailing filter min_angle" << std::endl;
        return false;
    }

    if (!tf->GetDouble("max_angle", filters.trailing_filter.max_angle))
        return false;

    if (filters.trailing_filter.max_angle < 0 || filters.trailing_filter.max_angle > 360) {
        std::cerr << "Illegal trailing filter max_angle" << std::endl;
        return false;
    }

    if (filters.trailing_filter.min_angle > filters.trailing_filter.max_angle) {
        std::cerr << "Illegal trailing filter min_angle and max_angle" << std::endl;
        return false;
    }

    if (!configurator->GetBool("use_angle_filter", filters.angle_filter.enable))
        return false;

    auto af = configurator->GetExtractor("angle_filter");
    if (af == nullptr)
        return false;

    auto range_extractor = af->GetExtractor("range");
    if (range_extractor == nullptr)
        return false;

    std::vector<std::unique_ptr<Configurator>> extractors;
    if (!range_extractor->GetArray(extractors))
        return false;

    for (const auto &ext : extractors) {
        std::vector<double> array;
        if (!ext->GetArray(array))
            return false;

        if (array.size() != 2) {
            std::cerr << "Illegal angle filter parameters" << std::endl;
            return false;
        }

        if (array[0] > array[1]) {
            std::cerr << "Illegal angle filter parameters" << std::endl;
            return false;
        }

        if (array[0] < 0 || array[1] > 360) {
            std::cerr << "Illegal angle filter parameters" << std::endl;
            return false;
        }

        filters.angle_filter.removed.emplace_back(std::tuple<double, double>{array[0], array[1]});
    }

    return true;
}

std::vector<std::tuple<double, double>> GetRangeWithPose(const std::vector<std::tuple<double, double>> &range, double rotate)
{
    std::vector<std::tuple<double, double>> removed;
    for (size_t i = 0; i < range.size(); i++) {
        double begin, end;
        std::tie(begin, end) = range.at(i);

        begin += rotate;
        end += rotate;

        if (end <= 360) {
            removed.emplace_back(std::make_tuple(begin, end));
            continue;
        }

        if (begin >= 360) {
            removed.emplace_back(std::make_tuple(begin - 360, end - 360));
            continue;
        }

        if ((begin < 360) && (end > 360)) {
            removed.emplace_back(std::make_tuple(begin, 360));
            removed.emplace_back(std::make_tuple(0, end - 360));
        }
    }

    return removed;
}

bool GetLidarAttr(std::unique_ptr<Configurator> configurator, LidarAttribute &attribute)
{
    if (!configurator->GetDouble("max_range", attribute.max_range))
        return false;

    if (!configurator->GetDouble("zero_angle", attribute.zero_angle))
        return false;

    return true;
}


}