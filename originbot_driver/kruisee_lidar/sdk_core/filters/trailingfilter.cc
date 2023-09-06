//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "filters/trailingfilter.h"

#include <vector>
#include <cmath>
#include <iostream>

std::unique_ptr<TrailingFilter> TrailingFilter::Create(int windows, int neighbors, double min_angle, double max_angle)
{
    if (windows <= 0 || neighbors <= 0)
        return nullptr;

    if (min_angle < 0 || min_angle > 360 || max_angle < 0 || max_angle > 360 || max_angle < min_angle)
        return nullptr;

    std::unique_ptr<TrailingFilter> filter(new TrailingFilter(windows, neighbors, min_angle, max_angle));

    return filter;
}

TrailingFilter::TrailingFilter(int windows, int neighbors, double min_angle, double max_angle)
    : window_(windows), neighbors_(neighbors), min_angle_(min_angle), max_angle_(max_angle) { }

double TrailingFilter::GetAngleWithViewpoint(float r1, float r2, float included_angle)
{
	return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
}

bool TrailingFilter::Run(const LScan &scan, std::vector<int>& removed)
{
    std::vector<int> dflag;
    dflag.resize(scan.ranges.size());

    for (size_t i = 0; i <dflag.size(); i++)
        dflag[i] = 0;

    int count = scan.ranges.size();

    //std::set<int> indices_to_delete;
    float dandiao1, dandiao2;
    //indices_to_delete.clear();
    // For each point in the current line scan
    for (unsigned int i = 0; i < count; i++) {
        if (scan.ranges[i] == 0) {
            continue;
        }

        if (-1 == dflag[i]) {
            continue;
        }

        for (int y = -window_; y < window_ + 1; y++) {
            int j = i + y;
            if (j < 0 || j >= count || (int)i == j) {
                // Out of scan bounds or itself
                continue;
            }

            dandiao1 = 1;
            dandiao2 = 1;
            double angle1, angle2, angle;
            angle1 = GetAngleWithViewpoint(float(scan.ranges[i]), float(scan.ranges[j]), y * 0.25 * M_PI / 180);
            angle2 = angle1 * 180 / M_PI;
            angle = std::abs(angle2);
            if (j < i) {
                if (j == (i - 1)) {
                    dandiao1 = dandiao1*angle1;
                }

            } else {
                if (j == (i + 1)) {
                    dandiao2 = dandiao2*angle1;
                }
            }

            if (angle < min_angle_ || angle > max_angle_) {
#if 0
                for (int index = std::max<int>(i - neighbors_, 0); index <= std::min<int>(i + neighbors_, count - 1); index++) {
                    if (copy_ptr_org[i].distance < copy_ptr_org[index].distance)
                        indices_to_delete.insert(index);
                }
#else
                for (int index = 0 - neighbors_; index <= neighbors_; index++) {
                    int k;
                    if (index == 0)
                        continue;

                    k = i + index;
                    while (1) {
                        if ((k < 0))
                            break;

                        if ((k > (count - 1)))
                            break;

                        if (scan.ranges[k]) {
                            float distance_std = (scan.ranges[i]+scan.ranges[k])*0.020;
                            // delete neighbor if they are farther away (note not self)
                            if ((scan.ranges[i] + distance_std) < scan.ranges[k]) {
                                dflag[i] = -1;
                            }
                            break;
                        }

                        if (index > 0)
                            k++;

                        if (index < 0)
                            k--;
                    }//end while
                }
#endif
            }
        }

        if ((dandiao1*dandiao2) < 0) {
            for (int index = std::max<int>(i - 1, 0); index <= std::min<int>(i + 1, count - 1); index++) {
                //indices_to_delete.erase(index);
                dflag[index] = 0;
            }
        }
    }

    for (size_t i = 0; i < dflag.size(); i++) {
        if (-1 == dflag[i])
            removed.push_back(i);
    }

    return true;
}