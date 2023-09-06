//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#ifdef _WIN32
#include <Windows.h>
#endif

#include <memory>
#include <atomic>

#include "drivers/iworker.h"

class Uart : public IWorker {
public:
    static std::unique_ptr<Uart> Create(const std::string &name, uint32_t baudrate);
    ~Uart();

    bool Launch(Handler handler, bool detached) override;
    bool Write(const std::string &cmd) override;
    bool GetCtlRun() override;
    bool SetCtlRunHandler(CtlRunHandler handler) override;
    bool SetErrHandler(ErrHandler handler) override;
    void SetPack(bool p, double t) override;

private:
    Uart();
    bool Init(const std::string &name, uint32_t baudrate);

    std::string uart_name;
    uint32_t uart_baudrate;

#ifdef _WIN32
    void Handle(HANDLE fd);
#elif __linux__
    void Handle(int fd);
#else
#error This platform is not supported temporarily
#endif

private:
#ifdef _WIN32
    HANDLE fd_;
    OVERLAPPED overlapped_;
#elif __linux__
    int fd_;
#endif

    Handler handler_;
    CtlRunHandler ctl_run_handler_;
    ErrHandler err_handler_;
    bool is_pack;

    std::atomic<bool> quit_;
    std::atomic<bool> try_to_quit_;
    uint64_t last_err_time_;
    double last_pub_time_;
};