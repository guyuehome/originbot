//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#ifdef _WIN32

#include "drivers/uart.h"

#include <Windows.h>
#include <winnt.h>

#include <sstream>
#include <iostream>
#include <thread>

namespace {

// 设置串口发送接收缓冲大小
constexpr uint32_t kRWBufferSize = 4096;

} // namespace

Uart::Uart()
    : fd_(INVALID_HANDLE_VALUE), overlapped_{0}, quit_(false), try_to_quit_(false) {}

Uart::~Uart()
{
    try_to_quit_ = true;
    while (!quit_);

    if (fd_ != INVALID_HANDLE_VALUE)
        ::CloseHandle(fd_);

    if (overlapped_.hEvent != INVALID_HANDLE_VALUE)
        ::CloseHandle(overlapped_.hEvent);
}

std::unique_ptr<Uart> Uart::Create(const std::string &name, uint32_t baudrate)
{
    std::unique_ptr<Uart> uart(new Uart());
    if (uart == nullptr)
        return nullptr;

    if (!uart->Init(name, baudrate))
        return nullptr;

    return uart;
}

bool Uart::Launch(Handler handler, bool detached)
{
    this->handler_ = handler;

    if (fd_ < 0) {
        std::cerr << "Failed to find available uart" << std::endl;
        return false;
    }

    std::thread t(&Uart::Handle, this, fd_);

    if (detached) {
        t.detach();
    } else {
        t.join();
    }

    return true;
}

bool Uart::Write(const std::string &cmd)
{
    if (fd_ == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to found avilable uart" << std::endl;
        return false;
    }

    DWORD err;
    COMSTAT stat;
    ::ClearCommError(fd_, &err, &stat);

    OVERLAPPED overlapped;
    ::ZeroMemory(&overlapped, sizeof(overlapped));
    memset(&overlapped, 0, sizeof(overlapped));
    overlapped.hEvent = ::CreateEvent(nullptr, true, false, nullptr);

    bool finished = true;
    DWORD length = cmd.length();
    if (!::WriteFile(fd_, cmd.c_str(), length, &length, &overlapped)) {
        if (::GetLastError() == ERROR_IO_PENDING) {
            ::WaitForSingleObject(overlapped.hEvent, 1000);
        } else {
            finished = false;
            std::cerr << "Failed to Write data" << std::endl;
        }
    }

    ::PurgeComm(fd_, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

    return finished;
}

bool Uart::Init(const std::string &name, uint32_t baudrate)
{
    overlapped_.hEvent = ::CreateEvent(nullptr, false, false, nullptr);
    if (overlapped_.hEvent == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to create event" << std::endl;
        return false;
    }

    auto tr = [] (const std::string &name) -> const char * {
        return std::string("\\\\.\\").append(name).c_str();
    };

    // [1] open device
    HANDLE fd = ::CreateFile(tr(name), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                             FILE_FLAG_OVERLAPPED, nullptr);
    if (fd == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to open device: " << name << std::endl;
        return false;
    }

    // [2] configure device
    ::SetupComm(fd, kRWBufferSize, kRWBufferSize);
    COMMTIMEOUTS timeouts;
    memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;

    if (!::SetCommTimeouts(fd, &timeouts)) {
        std::cerr << "Failed to timeout parameters" << std::endl;
        return false;
    }

    DCB dcb;
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);

    if (!::GetCommState(fd, &dcb)) {
        std::cerr << "Failed to get dcb" << std::endl;
        return false;
    }

    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (!::SetCommState(fd, &dcb)) {
        std::cerr << "Failed to set serialport parameters" << std::endl;
        return false;
    }

    if (!::SetCommMask(fd, EV_RXCHAR | EV_ERR)) {
        std::cerr << "Failed to set event mask for RX and error events" << std::endl;
        return false;
    }

    this->fd_ = fd;
    return true;
}

void Uart::Handle(HANDLE fd)
{
    DWORD err;
    COMSTAT stat = { 0 };

    DWORD bytes = 0;
    uint8_t buffer[kRWBufferSize];
    uint32_t index = 0;

    auto Read = [&] (uint8_t *data, int length) -> int {
        if (length >= bytes) {
            memcpy(data, buffer, bytes);
            index = 0;
            return bytes;
        } else {
            memcpy(data, buffer, length);

            for (int i = 0; i < bytes - length; i++)
                *(buffer + i) = *(buffer + length + i);

            index = length;
            return length;
        }
    };

    while (true) {
        ::ClearCommError(fd, &err, &stat);

        if (try_to_quit_)
            break;

        overlapped_.Offset = 0;
        bool ret = ::ReadFile(fd, &buffer[index], sizeof(buffer) - index, &bytes, &overlapped_);
        if (ret == false && ::GetLastError() != ERROR_IO_PENDING) {
            std::cerr << "Failed to read data from device" << std::endl;
            break;
        }

        ret = ::GetOverlappedResult(fd, &overlapped_, &bytes, true);
        if (ret != false && bytes > 0)
            this->handler_(Read);

        ::PurgeComm(fd, PURGE_RXCLEAR | PURGE_RXABORT);
    }

    quit_ = true;
}

#endif