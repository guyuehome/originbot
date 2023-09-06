//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <string>
#include <functional>

using Handler = std::function<void(std::function<int(uint8_t *, int)>)>;
using CtlRunHandler = std::function<bool()>;
using WorkCmdHandler = std::function<bool(const std::string&)>;
using ErrHandler = std::function<void(const std::string&)>;

class IWorker {
public:
    virtual ~IWorker() = default;

    virtual bool Launch(Handler handler, bool detached) = 0;
    virtual bool Write(const std::string &cmd) = 0;
    virtual bool GetCtlRun() = 0;
    virtual bool SetCtlRunHandler(CtlRunHandler handler) = 0;
    virtual bool SetErrHandler(ErrHandler handler) = 0;
    virtual void SetPack(bool p, double t) = 0;
};