//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "sdkcore.h"

#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#include "common/configurator.h"
#include "common/scripts.h"
#include "drivers/uart.h"
#include "drivers/ethernet.h"
#include "drivers/lidar.h"

namespace {

void ScanCopy(const LScan &scan, LScan &new_scan)
{
    new_scan.stamp = scan.stamp;
    new_scan.interval = scan.interval;

    new_scan.angle_increment = scan.angle_increment;

    new_scan.min_angle = scan.min_angle;
    new_scan.max_angle = scan.max_angle;

    new_scan.min_range = scan.min_range;
    new_scan.max_range = scan.max_range;

    new_scan.ranges.resize(scan.ranges.size());
    new_scan.intensities.resize(scan.intensities.size());
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        new_scan.ranges[i] = scan.ranges[i];
        new_scan.intensities[i] = scan.intensities[i];
    }
}

void SwitchToCartesian(LScan &scan, std::vector<Point2D> &points)
{
    if (points.size() != scan.ranges.size())
        points.resize(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); i++) {
        double angle = scan.min_angle + scan.angle_increment * i;
        points[i].x = scan.ranges[i] * cos(angle);
        points[i].y = scan.ranges[i] * sin(angle);
    }
}

} // namespace

std::unique_ptr<SdkCore> SdkCore::Create(const std::vector<std::string> &cfg_path, const std::string &main_file)
{
    if (cfg_path.empty() || main_file.empty()) {
        std::cerr << "Parameter cfg_path or main_file does not exist" << std::endl;
        return nullptr;
    }

    auto configurator = Configurator::Create(cfg_path, main_file);
    if (configurator == nullptr) {
        std::cerr << "Failed to create Configurator" << std::endl;
        return nullptr;
    }

    auto loader = configurator->GetExtractor("filters");
    if (loader == nullptr) {
        std::cerr << "Failed to load filter parameters" << std::endl;
        return nullptr;
    }

    Scripts::Filters params;
    if (!Scripts::GetFiltersOptions(std::move(loader), params))
        return nullptr;

    // 创建离群点滤波器
    Scripts::OFParams of = params.outlier_filter;
    std::unique_ptr<OutlierFilter> outlierfilter = OutlierFilter::Create(of.radius, of.min_count, of.max_distance);
    outlierfilter->set_min_distance(0);
    if (outlierfilter == nullptr) {
        std::cerr << "Failed to create Radius Filter" << std::endl;
        return nullptr;
    }

    Scripts::OFParams ofn = params.outlier_filter_near;
    std::unique_ptr<OutlierFilter> outlierfilter_near = OutlierFilter::Create(ofn.radius, ofn.min_count, ofn.max_distance);
    outlierfilter_near->set_min_distance(0);
    if (outlierfilter_near == nullptr) {
        std::cerr << "Failed to create Radius Filter near" << std::endl;
        return nullptr;
    }

    // 创建平滑滤波器
    Scripts::SFParams sf = params.smooth_filter;
    std::unique_ptr<Smoother> smoother = Smoother::Create(sf.level, sf.err);
    if (smoother == nullptr) {
        std::cerr << "Failed to Create Smoother" << std::endl;
        return nullptr;
    }

    // 创建拖尾滤波器
    Scripts::TFParams tf = params.trailing_filter;
    std::unique_ptr<TrailingFilter> trailingfilter = TrailingFilter::Create(tf.window, tf.neighbors, tf.min_angle, tf.max_angle);
    if (trailingfilter == nullptr) {
        std::cerr << "Failed to Create TrailingFilter" << std::endl;
        return nullptr;
    }

    loader = configurator->GetExtractor("protocol");
    if (loader == nullptr) {
        std::cerr << "Failed to load protocol parameters" << std::endl;
        return nullptr;
    }

    Scripts::Protocol protocol;
    if (!Scripts::GetProtocol(std::move(loader), protocol))
        return nullptr;

    if (!(protocol.uart.enable ^ protocol.ethernet.enable)) {
        std::cerr << "The communication protocol is not set or set twice" << std::endl;
        return nullptr;
    }

    std::unique_ptr<IWorker> worker = nullptr;
    if (protocol.ethernet.enable) {
        worker = Ethernet::Create(protocol.ethernet.listen_port, protocol.ethernet.radar_ip, protocol.ethernet.radar_port);
    } else {
        worker = Uart::Create(protocol.uart.name, protocol.uart.baudrate);
    }

    if (worker == nullptr) {
        std::cerr << "Failed to Create Ethernet or Uart" << std::endl;
        return nullptr;
    }

    std::unique_ptr<Lidar> lidar = Lidar::Create(std::move(worker));
    if (lidar == nullptr) {
        std::cerr << "Failed to Create Lidar" << std::endl;
        return nullptr;
    }

    loader = configurator->GetExtractor("lidar");
    if (loader == nullptr) {
        std::cerr << "Failed to load lidar parameters" << std::endl;
        return nullptr;
    }

    Scripts::LidarAttribute attr;
    if (!Scripts::GetLidarAttr(std::move(loader), attr))
        return nullptr;

    lidar->SetAttribute(attr);

    if (params.angle_filter.enable)
        lidar->SetRemoved(Scripts::GetRangeWithPose(params.angle_filter.removed, attr.zero_angle));

    std::unique_ptr<SdkCore> sdk(new SdkCore(std::move(lidar)));
    if (sdk == nullptr)
        return nullptr;

    if (of.enable)
    {
        if (ofn.enable)
        {
            outlierfilter->set_min_distance(ofn.max_distance);
        }

        outlierfilter->set_search_flags(0, 1440);
        sdk->set_outlier_filter(std::move(outlierfilter));
    }

    sdk->outlier_filter_near_params_.front_angle = params.outlier_filter_near_angle.front_angle;
    sdk->outlier_filter_near_params_.side_angle = params.outlier_filter_near_angle.side_angle;
    sdk->outlier_filter_near_params_.lidar_angle = params.outlier_filter_near_angle.lidar_angle;
    sdk->outlier_filter_near_params_.angle = params.outlier_filter_near_angle.angle;

    sdk->set_outlier_filter_near_angles(params.outlier_filter_near_angle.front_angle,
                                    params.outlier_filter_near_angle.side_angle, 
                                    params.outlier_filter_near_angle.lidar_angle);

    if(sdk->outlier_filter_near_params_.angle)
    {
        for (const auto &range : sdk->outlier_filter_near_angles_) {
            int begin = std::get<0>(range);
            int end = std::get<1>(range);
            //std::cout << "range:" << begin << "," << end << std::endl;
            outlierfilter_near->set_search_flags(begin, end);
        }
    }
    else
    {
        outlierfilter_near->set_search_flags(0, 1440);
    }

    if (ofn.enable)
    {
        sdk->set_outlier_filter_near(std::move(outlierfilter_near));
    }     

    if (sf.enable)
        sdk->set_smoother(std::move(smoother));

    if (tf.enable)
        sdk->set_trailing_filter(std::move(trailingfilter));

    return sdk;
}

void SdkCore::set_outlier_filter_near_angles(double front_angle, double side_angle, double lidar_angle)
{
    int front_2 = static_cast<int>(front_angle / 0.5);
    int side = static_cast<int>(side_angle / 0.25);
    int lidar = static_cast<int>(lidar_angle / 0.25);

    int cs = -1;

    if( (lidar >= 0) && (lidar < front_2) )
    {
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(1440+lidar-side-front_2), (1440+lidar-front_2)});
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(lidar+front_2), (lidar+front_2+side)});
        cs = 1;
    }
    else if( (lidar >= front_2) && (lidar < (front_2+side)) )
    {
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(1440+lidar-side-front_2), 1440});
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{0, (lidar-front_2)});
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(lidar+front_2), (lidar+front_2+side)});
        cs = 2;
    }
    else if( (lidar >= (front_2+side)) && (lidar < (1440-side-front_2)) )
    {
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(lidar-side-front_2), (lidar-front_2)});
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(lidar+front_2), (lidar+front_2+side)});
        cs = 3;
    }
    else if( (lidar >= (1440-side-front_2)) && (lidar < (1440-front_2)) )
    {
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(lidar-side-front_2), (lidar-front_2)});
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(lidar+front_2), 1440});
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{0, (lidar+front_2+side-1440)});
        cs = 4;
    }
    else if( (lidar >= (1440-front_2)) && (lidar < 1440) )
    {
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(lidar-side-front_2), (lidar-front_2)});
        outlier_filter_near_angles_.emplace_back(std::tuple<int, int>{(lidar+front_2-1440), (lidar+front_2+side-1440)});
        cs = 5;
    }

    /*
    std::cout << "range type:" << cs << std::endl;

    if (!outlier_filter_near_angles_.empty()) {
        for (const auto &range : outlier_filter_near_angles_) {
            int begin = std::get<0>(range);
            int end = std::get<1>(range);
            std::cout << "range:" << begin << "," << end << std::endl;
        }
    }
    */
}

SdkCore::SdkCore(std::unique_ptr<Lidar> lidar)
    : lidar_(std::move(lidar)), handler_{nullptr}, outlier_filter_(nullptr), smoother_(nullptr),
      trailing_filter_(nullptr)
{
    lidar_->Bind(std::bind(&SdkCore::Handle, this, std::placeholders::_1));
}

SdkCore::~SdkCore() { }

void SdkCore::RegisterDataDistributor(Type type, std::function<void(const LScan &)> handler)
{
    if (static_cast<uint32_t>(type) >= 2) {
        std::cerr << "Failed to Register DataDistributor" << std::endl;
        return;
    }

    handler_[static_cast<uint32_t>(type)] = handler;
}

void SdkCore::RegisterCtlRunHandler(CtlRunHandler handler)
{
    lidar_->SetWorkerCtlRunHandler(handler);
}

void SdkCore::RegisterErrHandler(ErrHandler handler)
{
    lidar_->SetWorkerErrHandler(handler);
}

void SdkCore::Publish(Type type, const LScan &scan)
{
    if (static_cast<uint32_t>(type) >=  2) {
        std::cerr << "Failed to Publish data to distributor" << std::endl;
        return;
    }

    if (handler_[static_cast<uint32_t>(type)] != nullptr)
        handler_[static_cast<uint32_t>(type)](scan);
}

bool SdkCore::Run(bool detached)
{
    return lidar_->Launch(detached);
}

bool SdkCore::Write(const std::string &cmd)
{
    return lidar_->Write(cmd);
}

void SdkCore::Handle(LScan &scan)
{
    this->Publish(Type::Raw, scan);

    if(scan.orig)
    {
        this->Publish(Type::Filtered, scan);
        return;
    }

    LScan new_scan;
    if (outlier_filter_ != nullptr) 
    {
        ScanCopy(scan, new_scan);
    }

    //LScan new_scan_near;
    //ScanCopy(scan, new_scan_near);

    if (smoother_ != nullptr) {
        smoother_->SetData(&scan.ranges);
        smoother_->Run(scan.ranges);
    }

    if (trailing_filter_ != nullptr) {
        std::vector<int> removed;
        trailing_filter_->Run(scan, removed);

        for (auto index : removed)
            scan.ranges[index] = 0;
    }


/*
    //yuanlai
    if (outlier_filter_ != nullptr) {
        std::vector<Point2D> points;
        SwitchToCartesian(new_scan, points);

        outlier_filter_->SetCloud(&points);

        std::vector<Point2D> handled;
        std::vector<int> removed;
        if (!outlier_filter_->Run(OutlierFilter::Mode::Knn, handled, removed)) {
            std::cerr << "Failed to filter outlier" << std::endl;
            return;
        }

        for (size_t i = 0; i < removed.size(); i++) {
            new_scan.ranges[removed.at(i)] = 0;
            new_scan.intensities[removed.at(i)] = 0;
        }

        for (size_t i = 0; i < scan.ranges.size(); i++) {
            if (0 != new_scan.ranges[i])
                scan.ranges[i] = new_scan.ranges[i];
        }
    }
*/

    if (outlier_filter_ != nullptr) {
        //far
        std::vector<Point2D> points;
        SwitchToCartesian(new_scan, points);

        outlier_filter_->SetCloud(&points);
        outlier_filter_->SetLScan(&new_scan);

        std::vector<Point2D> handled;
        std::vector<int> removed;
        if (!outlier_filter_->Run(OutlierFilter::Mode::Knn, handled, removed)) {
            std::cerr << "Failed to filter outlier" << std::endl;
            return;
        }

        for (size_t i = 0; i < removed.size(); i++) {
            new_scan.ranges[removed.at(i)] = 0;
            new_scan.intensities[removed.at(i)] = 0;
        }

        for (size_t i = 0; i < scan.ranges.size(); i++) {
            //if (0 != new_scan.ranges[i])
            if( (scan.ranges[i] < outlier_filter_->get_max_distance()) && (new_scan.ranges[i] < outlier_filter_->get_max_distance()) )
            {
                scan.ranges[i] = new_scan.ranges[i];
            }
        }
    }

    if (outlier_filter_near_ != nullptr) {
        //near
        LScan new_scan_near;
        ScanCopy(scan, new_scan_near);

        std::vector<Point2D> points_near;
        SwitchToCartesian(new_scan_near, points_near);

        outlier_filter_near_->SetCloud(&points_near);
        outlier_filter_near_->SetLScan(&new_scan_near);
        
        std::vector<Point2D> handled_near;
        std::vector<int> removed_near;
        if (!outlier_filter_near_->Run(OutlierFilter::Mode::Knn, handled_near, removed_near)) {
            std::cerr << "Failed to filter outlier" << std::endl;
            return;
        }
        
        for (size_t i = 0; i < removed_near.size(); i++) {
            new_scan_near.ranges[removed_near.at(i)] = 0;
            new_scan_near.intensities[removed_near.at(i)] = 0;
        }

        if(outlier_filter_near_params_.angle)
        {
            if (!outlier_filter_near_angles_.empty()) {
                for (const auto &range : outlier_filter_near_angles_) {
                    int begin = std::get<0>(range);
                    int end = std::get<1>(range);
                    //std::cout << "range:" << begin << "," << end << std::endl;
                    
                    for (size_t i = begin; i < end; i++) {
                        if( (scan.ranges[i] < outlier_filter_near_->get_max_distance()) && (new_scan_near.ranges[i] < outlier_filter_near_->get_max_distance()) )
                        {
                            scan.ranges[i] = new_scan_near.ranges[i];
                        }
                    }
                }
            }
        }
        else
        {
            for (size_t i = 0; i < scan.ranges.size(); i++) {
                if( (scan.ranges[i] < outlier_filter_near_->get_max_distance()) && (new_scan_near.ranges[i] < outlier_filter_near_->get_max_distance()) )
                {
                    scan.ranges[i] = new_scan_near.ranges[i];
                }
            }
        }
        
    }

/*
    if (outlier_filter_ != nullptr) 
    {
        std::vector<Point2D> points;
        SwitchToCartesian(new_scan, points);

        outlier_filter_->SetCloud(&points);

        std::vector<Point2D> handled;
        std::vector<int> removed;
        if (!outlier_filter_->Run(OutlierFilter::Mode::Knn, handled, removed)) {
            std::cerr << "Failed to filter outlier" << std::endl;
            return;
        }

        for (size_t i = 0; i < removed.size(); i++) {
            scan.ranges[removed.at(i)] = 0;
        }
    }
*/

    if (smoother_ != nullptr) {
        smoother_->SetData(&scan.ranges);
        smoother_->Run(scan.ranges, scan.intensities);
    }

    this->Publish(Type::Filtered, scan);
}
