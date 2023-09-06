//
// Copyright (c) 2021 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <stdint.h>

#include <vector>

struct LScan {
    double stamp;           // time stamp
    double interval;        // time interval between two adjacent frames

    float angle_increment;  // angle increment(amplitude)
    float min_angle;        // minimum angle(amplitude)
    float max_angle;        // maximum angle(amplitude)

    float min_range;        // minimum distance(m)
    float max_range;        // maximum distance(m)

    float time_increment;   // time increment(sec)

    std::vector<float> ranges;      // distance array
    std::vector<float> intensities; // intensity array

    bool orig;
};