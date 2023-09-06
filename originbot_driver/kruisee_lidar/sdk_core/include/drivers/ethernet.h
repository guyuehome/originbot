//
// Copyright (c) 2021 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <memory>
#include <atomic>
#include <string>

#include "drivers/iworker.h"

class Ethernet : public IWorker {
public:
    static std::unique_ptr<Ethernet> Create(uint16_t listen_port, const std::string &ip, uint16_t port);
    ~Ethernet();

    bool Launch(Handler handler, bool detached) override;
    bool Write(const std::string &cmd) override;
    bool GetCtlRun() override;
    bool SetCtlRunHandler(CtlRunHandler handler) override;
    bool SetErrHandler(ErrHandler handler) override;
    void SetPack(bool p, double t) override;

private:
    Ethernet();
    bool Init(uint16_t listen_port);
    void Handle(int fd);
    bool Write(const char* cmd, size_t len);

private:
    std::string ip_;
    uint16_t port_;

    CtlRunHandler ctl_run_handler_;
    ErrHandler err_handler_;
    bool is_pack;

    int fd_;
    Handler handler_;

    std::atomic<bool> quit_;
    std::atomic<bool> try_to_quit_;
};