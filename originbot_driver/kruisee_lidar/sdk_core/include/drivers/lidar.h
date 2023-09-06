//
// Copyright (c) 2021 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <memory>
#include <functional>
#include <tuple>
#include <vector>

#include "common/scripts.h"
#include "drivers/iworker.h"
#include "drivers/msg.h"
#include "drivers/protocol.h"

// 相对于模组0度旋转角度(逆时针)
#define DTOF_DEFAULT_ROTATE_ANGLE (123)

constexpr int kMaxBuffer = 4 * 1024;// 4Kb

class Lidar {
public:
    static std::unique_ptr<Lidar> Create(std::unique_ptr<IWorker> dev);
    ~Lidar();

    void Bind(std::function<void(LScan &)> caller) { this->caller_ = caller; }
    bool Launch(bool detached = true);

    bool Write(const std::string &cmd);

    void SetAttribute(const Scripts::LidarAttribute &attr) { attr_ = attr; }
    void SetRemoved(const std::vector<std::tuple<double, double>> &removed) { this->removed_ = removed; }

    void SetWorkerCtlRunHandler(CtlRunHandler handler);

    void SetWorkerErrHandler(ErrHandler handler);

private:
    Lidar();

    void SetWorker(std::unique_ptr<IWorker> dev) { this->dev_ = std::move(dev); }
    void Handle(std::function<int(uint8_t *, int)> read);

    void ParseAndPackage(Packet *packet, bool energetic);
    double Now();

private:
    std::unique_ptr<IWorker> dev_;
    std::function<void(LScan &)> caller_;

    std::vector<std::tuple<double, double>> removed_;

    uint8_t buffer_[kMaxBuffer];
    int xfer_;

    std::vector<float> ranges_;
    std::vector<float> intensities_;

    bool started_;
    uint8_t last_angle_;

    double last_stamp_;

    double rotate_angle_;
    Scripts::LidarAttribute attr_;
};