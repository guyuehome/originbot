//
// Copyright (c) 2021 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#ifdef _WIN32

#include "drivers/ethernet.h"

#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <sys/types.h>
#include <WinSock2.h>
#include <windows.h>

#include <chrono>
#include <iostream>
#include <thread>

namespace {

// 底层 UDP 数据缓冲大小
constexpr uint32_t kBufferSize = 4096;

} // namespace

Ethernet::Ethernet()
    : ip_(), port_(0), fd_(-1), handler_(nullptr), quit_(false), try_to_quit_(false) {}

Ethernet::~Ethernet()
{
    try_to_quit_ = true;
    while (!quit_);

    if (fd_ < 0)
        return;

    closesocket(fd_);
    WSACleanup();
}

std::unique_ptr<Ethernet> Ethernet::Create(uint16_t listen_port, const std::string &ip, uint16_t port)
{
    std::unique_ptr<Ethernet> ethernet(new Ethernet());
    if (ethernet == nullptr)
        return nullptr;

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

    std::thread t(&Ethernet::Handle, this, fd_);

    if (detached) {
        t.detach();
    } else {
        t.join();
    }

    return true;
}

bool Ethernet::Init(uint16_t listen_port)
{
    WSADATA wsadata;
    if (WSAStartup(MAKEWORD(2, 2), &wsadata) != 0) {
        std::cerr << "Failed to initialize socket dll" << std::endl;
        return false;
    }

    fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        WSACleanup();
        return false;
    }

    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(listen_port);

    if (bind(fd_, (struct sockaddr *)&local, sizeof(local)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind port" << std::endl;
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

void Ethernet::Handle(int fd)
{
    struct sockaddr_in client;
    int length = sizeof(SOCKADDR);

    char buffer[kBufferSize];
    uint32_t index = 0;
    while (true) {
        if (try_to_quit_)
            break;

        int ret = ::recvfrom(fd, &buffer[index], sizeof(buffer) - index, 0, (SOCKADDR *)&client, &length);
        if (ret < 0) {
            std::cerr << "Failed to receive data from client" << std::endl;
            continue;
        }

        auto Read = [&] (uint8_t *data, int length) -> int {
            if (length >= ret) {
                memcpy(data, buffer, ret);
                index  = 0;
                return ret;
            } else {
                memcpy(data, buffer, length);

                for (int i = 0; i < ret - length; i++)
                    *(buffer + i) = *(buffer + length + i);

                index = length;
                return length;
            }
        };

        this->handler_(Read);
    }

    quit_ = true;
}

#endif