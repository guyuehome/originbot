//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <vector>
#include <memory>

#include "filters/ismoother.h"

class Smoother : public ISmoother {
public:
    static std::unique_ptr<Smoother> Create(int level, float err);
    ~Smoother() = default;

    void SetData(std::vector<float> *data) override { this->data_ = data; }
    bool Run(std::vector<float> &out) override;
    bool Run(std::vector<float> &out, std::vector<float> &inten) override;

private:
    Smoother(int level, float err) : level_(level), err_(err), data_(nullptr) {}
    float Submap(float *data, size_t length);

private:
    int level_;
    float err_;
    std::vector<float> *data_;
};