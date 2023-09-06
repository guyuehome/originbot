//
// Copyright (c) 2021 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#ifdef __linux__

#include "drivers/ethernet.h"

#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <sys/types.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <arpa/inet.h>

#include <iostream>
#include <thread>

Ethernet::Ethernet()
    : ip_(), port_(0), fd_(-1), handler_(nullptr), quit_(false), try_to_quit_(false) {}

Ethernet::~Ethernet()
{
    try_to_quit_ = true;
    while (!quit_);

    if (fd_ < 0)
        return;

    close(fd_);
}

std::unique_ptr<Ethernet> Ethernet::Create(uint16_t listen_port, const std::string &ip, uint16_t port)
{
    std::unique_ptr<Ethernet> ethernet(new Ethernet());
    if (ethernet == nullptr)
        return nullptr;

    ethernet->ip_ = ip;
    ethernet->port_ = port;

    if (!ethernet->Init(listen_port))
        return nullptr;

    return ethernet;
}

bool Ethernet::Launch(Handler handler, bool detached)
{
    this->handler_ = handler;
    if (fd_ < 0) {
        std::cerr << "Failed to find available socket" << std::endl;
        return false;
    }

    // [1] create epoll object
    int fd = epoll_create(1);

    // [2] add listen object
    struct epoll_event event;
    event.events = EPOLLIN;
    event.data.fd = fd_;

    if (epoll_ctl(fd, EPOLL_CTL_ADD, fd_, &event) < 0) {
        std::cerr << "Failed to add monitored object" << std::endl;
        return false;
    }

    std::thread t(&Ethernet::Handle, this, fd);

    if (detached) {
        t.detach();
    } else {
        t.join();
    }

    return true;
}

bool Ethernet::Init(uint16_t listen_port)
{
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::cerr << "Failed to create socket." << std::endl;
        return false;
    }

    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(listen_port);

    if (bind(fd_, (struct sockaddr *)&local, sizeof(local)) < 0) {
        std::cerr << "Failed to bind port." << std::endl;
        return false;
    }

    return true;
}

bool Ethernet::Write(const std::string &cmd)
{
    if (fd_ < 0) {
        std::cerr << "Failed to find available socket" << std::endl;
        return false;
    }

    struct sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(ip_.c_str());
    server.sin_port = htons(port_);
    uint32_t length = sizeof(server);

    int res = sendto(fd_, cmd.c_str(), cmd.length(), 0, (struct sockaddr *)&server, length);
    if (res < 0)
        return false;

    return true;
}

bool Ethernet::Write(const char* cmd, size_t len)
{
    if (fd_ < 0) {
        std::cerr << "Failed to find available socket" << std::endl;
        return false;
    }

    //std::cout << "ip:" << ip_ << std::endl;

    struct sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(ip_.c_str());
    server.sin_port = htons(port_);
    uint32_t length = sizeof(server);

    int res = sendto(fd_, cmd, len, 0, (struct sockaddr *)&server, length);
    //std::cout << "RES:" << res << std::endl;

    if (res < 0)
        return false;

    return true;
}

bool Ethernet::GetCtlRun()
{
    return ctl_run_handler_();
}

bool Ethernet::SetCtlRunHandler(CtlRunHandler handler)
{
    ctl_run_handler_ = handler;

    return true;
}

bool Ethernet::SetErrHandler(ErrHandler handler)
{
    err_handler_ = handler;
    return true;
}

void Ethernet::SetPack(bool p, double t) 
{
    is_pack = p;
}

void Ethernet::Handle(int fd)
{
    struct epoll_event events[1];

    while (true) {
        // [3] wait event
        int n = epoll_wait(fd, events, 1, 0.7*1000);
        if (n < 0) {
            if (try_to_quit_)
                break;

            std::cerr << "Failed to wait response" << std::endl;
            continue;
        }

        is_pack = false;

        if( (n > 0) && (events[0].events & EPOLLIN) ) {
            auto Read = [&](uint8_t *data, int length) -> int {
                return ::read(events[0].data.fd, data, length);
            };

            this->handler_(Read);
        }

        if(GetCtlRun())
        {
            if(!is_pack)
            {
                Write(std::string("startlds$"));
                //Write("startlds$", 9);
                //std::cout << "send startlds$ to lidar" << std::endl;
            }
        }
        else
        {
            if(is_pack)
            {
                Write(std::string("holdlds$"));
                //Write("holdlds$", 8);
                //std::cout << "send holdlds$ to lidar" << std::endl;
            }
        }

        if (try_to_quit_)
            break;
    }

    quit_ = true;
}

#endif
