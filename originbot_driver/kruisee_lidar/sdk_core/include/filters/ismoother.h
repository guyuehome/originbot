//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <vector>

class ISmoother {
public:
    ISmoother() = default;
    virtual ~ISmoother() = default;

    virtual void SetData(std::vector<float> *data) = 0;
    virtual bool Run(std::vector<float> &out) = 0;
    virtual bool Run(std::vector<float> &out, std::vector<float> &inten) = 0;
};