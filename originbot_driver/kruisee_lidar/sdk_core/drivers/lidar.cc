//
// Copyright (c) 2021 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "drivers/lidar.h"
#include "drivers/protocol.h"

#include <assert.h>
#include <sys/types.h>
#ifdef _WIN32
#include <WinSock2.h>
#include <windows.h>
#include <chrono>
#else
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <arpa/inet.h>
#endif

#include <iostream>

// 协议多字节数据是以小端形式组织的
// 这里根据用户系统大小端编译时自动进行设置
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define ntoh16(x)   (x)
#define ntoh32(x)   (x)
#else
#define ntoh16(x)   __bswap_16(x)
#define ntoh32(x)   __bswap_32(x)
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 一帧数据包含多个数据包,数据包是最小通信单位
// 该驱动主要是完成多多个数据包的接受并拼接,最后打包并以特定的方式发布给使用者


Lidar::Lidar()
    : caller_(nullptr), dev_(nullptr), xfer_(0), started_(false), last_angle_(0xA0), last_stamp_(INT32_MIN), attr_{INT16_MAX, 0}
{
    this->ranges_.resize(1440);
    this->intensities_.resize(1440);
}

Lidar::~Lidar() { }

std::unique_ptr<Lidar> Lidar::Create(std::unique_ptr<IWorker> worker)
{
    if (worker == nullptr)
        return nullptr;

    std::unique_ptr<Lidar> lidar(new Lidar());
    if (lidar == nullptr)
        return nullptr;

    lidar->SetWorker(std::move(worker));

    return lidar;
}

bool Lidar::Launch(bool detached)
{
    if (dev_ == nullptr) {
        std::cerr << "Failed to create lidar object" << std::endl;
        return false;
    }

    return dev_->Launch(std::bind(&Lidar::Handle, this, std::placeholders::_1), detached);
}

bool Lidar::Write(const std::string &cmd)
{
    if (dev_ == nullptr) {
        std::cerr << "Failed to create lidar object" << std::endl;
        return false;
    }

    return dev_->Write(cmd);
}

void Lidar::SetWorkerCtlRunHandler(CtlRunHandler handler)
{
    dev_->SetCtlRunHandler(handler);
}

void Lidar::SetWorkerErrHandler(ErrHandler handler)
{
    dev_->SetErrHandler(handler);
}

void Lidar::Handle(std::function<int(uint8_t *, int)> read)
{
    int pack_cnt = 0;
    // [1] 以追加的方式接收数据
    int length = read(&buffer_[xfer_], kMaxBuffer - xfer_);

    //std::cout << "length:" << length << std::endl;

    if (length <= 0)
        return;

    bool energetic = false;
    Packet *packet = reinterpret_cast<Packet *>(buffer_);
    while (reinterpret_cast<uint8_t *>(packet) + sizeof(*packet) < buffer_ + xfer_ + length) {
        // 2.1 识别协议帧头
        if (!Packet::IsHeader(packet->header))
            goto shifting;

        // 2.2 协议校验
        if (!Packet::IsValid(packet, energetic))
            goto shifting;

        // 2.3 数据转换并打包处理
        this->ParseAndPackage(packet, energetic);
        packet = reinterpret_cast<Packet *>(reinterpret_cast<uint8_t *>(packet) + Packet::length());
        pack_cnt += 1;
        continue;

shifting:
        packet = reinterpret_cast<Packet *>(reinterpret_cast<uint8_t *>(packet) + 1);
    }

    // [3] 每次接收并处理完后，将剩余数据移动到buffer_开始位置，用于解决数据越界问题
    int remain = reinterpret_cast<uint8_t *>(buffer_) + xfer_ + length - reinterpret_cast<uint8_t *>(packet);
    for (int i = 0; i < remain; i++)
        buffer_[i] = reinterpret_cast<uint8_t *>(packet)[i];

    xfer_ = remain;

    if(pack_cnt > 0)
    {
        dev_->SetPack(true, -1);
    }
}


#ifdef _WIN32
// 获取当前系统时间，如果是多机系统，用户需要提前完成系统的时钟同步工作
static int gettimeofday(struct timeval *__restrict __tv, void *__restrict __tz)
{
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    /*
     * A file time is a 64-bit value that represents the number of 100-nanosecond intervals that have elapsed
     * since January 1, 1601 12:00 A.M. UTC.
     *
     * Between January 1, 1970 (Epoch) and January 1, 1601 there were 134744 days, 11644473600 seconds or
     * 11644473600,000,000,0 100-nanosecond intervals.
     *
     * See also MSKB Q167296.
     * */
    uint64_t intervals = (static_cast<uint64_t>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
    intervals -= 116444736000000000;
    __tv->tv_sec = static_cast<long>(intervals / 10000000);
    __tv->tv_usec = static_cast<long>((intervals % 10000000) / 10);

    return 0;
}
#endif

double Lidar::Now()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    double time = tv.tv_sec + tv.tv_usec / 1000000.0;
    return time;
}

void Lidar::ParseAndPackage(Packet *packet, bool energetic)
{
    // 数据包连续性检测，防止某一帧数据丢失导致数据错乱
    if (packet->angle == 0xA0) {
        for (int i = 0; i < this->ranges_.size(); i++) {
            this->ranges_[i] = 0;
            this->intensities_[i] = 0;
        }

        started_ = true;
        last_angle_ = 0xA0;
    } else {
        // 如果帧内数据包不连续则放弃当前帧，并等待下一帧数据
        if (last_angle_ + 1 != packet->angle) {
            std::cerr << "loss" << std::endl;
            started_ = false;
        }
    }

    if (started_ == false)
        return;

    // 从数据包内解析距离数据并建立距离与角度的关系
    int index = (packet->angle - 0xa0) * 16;
    for (int i = 0; i < 16; i++) {
        float range = static_cast<float>(ntoh16(packet->data[i]) / 1000.0);
        if (range > attr_.max_range)
            range = 0.0;

        ranges_[index + i] = range;
        //intensities_[index + i] = energetic ? (packet->power[i] > 158 ? 255 : packet->power[i]) : 0;
        intensities_[index + i] = energetic ? (packet->power[i]) : 0;
    }

    last_angle_ = packet->angle;

    // 等待一帧数据接收完成，只有完成一帧数据的拼接才进行报文发布
    if (packet->angle != 0xf9)
        return ;

    started_ = false;

    LScan scan;
    // 为报文添加时间戳(数据接收完成时的时间戳)
    double now = Now();
    scan.stamp = now;
    dev_->SetPack(true, scan.stamp);
    if (last_stamp_ < 0) {
        scan.interval = 0;
        last_stamp_ = scan.stamp;
    } else {
        scan.interval = scan.stamp - last_stamp_;
        last_stamp_ = scan.stamp;
    }

    scan.angle_increment = static_cast<float>(0.25 * (M_PI / 180.0));
    scan.min_angle = 0;//-attr_.zero_angle * M_PI / 180.0;
    scan.max_angle = 2 * M_PI - scan.angle_increment;//static_cast<float>(2 * M_PI - scan.angle_increment) - attr_.zero_angle * M_PI / 180.0;
    scan.min_range = 0;
    scan.max_range = 25;

    double scan_time = 600.0/((double)(packet->speed));
    scan.time_increment = scan_time/1440.0;

    //std::cout << scan_time << "+++" << std::endl;
    //std::cout << scan.time_increment << "---" << std::endl;

    scan.ranges.resize(1440);
    scan.intensities.resize(1440);

    scan.ranges.swap(ranges_);
    scan.intensities.swap(intensities_);

    scan.orig = true;

    if (caller_ != nullptr) {
        caller_(scan);
    }

    scan.orig = false;

    // 检查是否需要进行角度滤波
    if (!removed_.empty()) {
        for (const auto &range : removed_) {
            int begin = static_cast<int>(std::get<0>(range) / 0.25);
            int end = static_cast<int>(std::get<1>(range) / 0.25);
            for (int i = begin; i < end + 1; i++) {
                scan.ranges[i] = 0;
                scan.intensities[i] = 0;
            }
        }
    }

    if (caller_ != nullptr) {
        caller_(scan);
    } else {
        std::cerr << "Please register the point cloud processing function!" << std::endl;
    }
}