//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <memory>
#include <vector>

#include "drivers/msg.h"

class TrailingFilter {
public:
    static std::unique_ptr<TrailingFilter> Create(int windows, int neighbors, double min_angle, double max_angle);
    ~TrailingFilter() = default;

    bool Run(const LScan &scan, std::vector<int> &removed);

private:
    TrailingFilter(int windows, int neighbors, double min_angle, double max_angle);
    double GetAngleWithViewpoint(float r1, float r2, float included_angle);

private:
    int window_ = 5;
    int neighbors_ = 3;
    double min_angle_ = 10;
    double max_angle_ = 170;
};