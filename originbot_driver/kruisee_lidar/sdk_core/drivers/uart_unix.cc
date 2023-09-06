//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#ifdef __linux__

#include "drivers/uart.h"

#include <termios.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <sys/time.h>

#include <map>
#include <thread>
#include <iostream>

static uint64_t get_time(void)
{
    struct timeval tv = {0};
    gettimeofday(&tv, NULL);

    uint64_t time = tv.tv_sec * 1e9 + tv.tv_usec * 1e3;
    return time;
}

const static std::map<int, int> kbaudrate_list = {
    { 57600, B57600 }, { 115200, B115200 }, { 230400, B230400 }, { 460800, B460800 }, { 500000, B500000 },
    { 576000, B576000 }, { 921600, B921600 }, { 1000000, B1000000 }, { 1152000, B1152000 }, { 1500000, B1500000 },
    { 2000000, B2000000 }, { 2500000, B2500000 }, { 3000000, B3000000 }, { 3500000, B3500000 },
    { 4000000, B4000000 },
};

Uart::Uart() : fd_(-1), quit_(false), try_to_quit_(false) 
{
    last_err_time_ = get_time()-5*1e9; 
    last_pub_time_ = last_err_time_/1e9;
}

Uart::~Uart()
{
    try_to_quit_ = true;
    while (!quit_);

    if (fd_ < 0)
        return;

    ::close(fd_);
}

std::unique_ptr<Uart> Uart::Create(const std::string &name, uint32_t baudrate)
{
    std::unique_ptr<Uart> uart(new Uart());
    if (uart == nullptr)
        return nullptr;

    uart->Init(name, baudrate);
    /*
    if (!uart->Init(name, baudrate))
        return nullptr;
    */
    return uart;
}

bool Uart::Launch(Handler handler, bool detached)
{
    this->handler_ = handler;

    int fd = -1;

    if (fd_ < 0) {
        std::cerr << "Failed to find available uart" << std::endl;
    }
    else
    {
        // [1] create epoll object
        fd = epoll_create(1);

        // [2] add listen object
        struct epoll_event event;
        event.events = EPOLLIN;
        event.data.fd = fd_;

        if (epoll_ctl(fd, EPOLL_CTL_ADD, fd_, &event) < 0) {
            std::cerr << "Failed to add monitored object" << std::endl;
            close(fd);
            fd = -1;
        }
    }

    std::thread t(&Uart::Handle, this, fd);

    if (detached) {
        t.detach();
    } else {
        t.join();
    }

    return true;
}

bool Uart::Write(const std::string &cmd)
{
    if (fd_ < 0) {
        std::cerr << "Failed to find available uart" << std::endl;
        return false;
    }

    int res = ::write(fd_, cmd.c_str(), cmd.length());
    if (res != cmd.length())
        return false;

    return true;
}

bool Uart::GetCtlRun()
{
    return ctl_run_handler_();
}

bool Uart::SetCtlRunHandler(CtlRunHandler handler)
{
    ctl_run_handler_ = handler;
    return true;
}

bool Uart::SetErrHandler(ErrHandler handler)
{
    err_handler_ = handler;
    return true;
}

void Uart::SetPack(bool p, double t) 
{
    is_pack = p;

    if(t > 0)
    {
        last_pub_time_ = t;
    }
}

bool Uart::Init(const std::string &name, uint32_t baudrate)
{
    this->uart_name = name;
    this->uart_baudrate = baudrate;

    if(460800 != baudrate)
    {
        std::cerr << "baudrate not 460800 to open device: " << name << std::endl;
        this->fd_ = -1;

        uint64_t cur_time = get_time();
        if( (cur_time-last_err_time_) > 5*1e9 )
        {
            if(err_handler_)
            {
                err_handler_(std::string("{\"code\":-1, \"desc\":\"串口波特率不正确应为460800\"}"));
            }

            last_err_time_ = cur_time;
        }

        return false;
    }

    // [1] open device
    int fd = ::open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Failed to open device: " << name << std::endl;
        fd = -1;
        this->fd_ = fd;
        return false;
    }

    // [2] configure device
    struct termios config;
    memset(&config, 0, sizeof(config));

    if (::tcgetattr(fd, &config) != 0) {
        std::cerr << "Failed to get serialport attribute" << std::endl;
        return false;
    }

    // set common props
    ::cfmakeraw(&config);
    config.c_cflag |= (CLOCAL | CREAD);
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN] = 0;

    // set databits => 8 bytes
    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;

    // set parity => no parity
    config.c_iflag &= ~(PARMRK | INPCK);
    config.c_iflag |= IGNPAR;
    config.c_cflag &= ~PARENB;

    // set stopbits => 1 byte
    config.c_cflag &= ~CSTOPB;

    // set flow control => no flow control
    config.c_cflag &= ~CRTSCTS;
    config.c_iflag &= ~(IXON | IXOFF | IXANY);

    // raw data output
    config.c_oflag &= ~OPOST;
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG /*| INLCR */);
    // config.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    config.c_iflag &= ~(IXON | IXOFF | IXANY | BRKINT | ICRNL | ISTRIP);

    if (::tcsetattr(fd, TCSANOW, &config) != 0) {
        std::cerr << "Failed to set attribute information" << std::endl;
        close(fd);
        return false;
    }

    // set baudrate
    if (kbaudrate_list.find(baudrate) == kbaudrate_list.end()) {
        std::cerr << "Failed to find the baudrate: " << baudrate << std::endl;
        ::close(fd);
        return false;
    }

    int br = kbaudrate_list.at(baudrate);
    ::cfsetispeed(&config, br);
    ::cfsetospeed(&config, br);

    if (::tcsetattr(fd, TCSANOW, &config) != 0) {
        std::cerr << "Faild to set baudrate" << std::endl;
        ::close(fd);
        return true;
    }

    ::tcflush(fd, TCIOFLUSH);

    if (fcntl(fd, F_SETFL, FNDELAY) != 0) {
        close(fd);
        return false;
    }

    this->fd_ = fd;
    return true;
}

void Uart::Handle(int fd_e)
{
    int fd = fd_e;
    struct epoll_event events[1];
    ::tcflush(fd_, TCIFLUSH);

    unsigned int unpack_cnt = 0;

open_uart:
    while (this->fd_ < 0)
    {
        uint64_t cur_time = get_time();
        if( (cur_time-last_err_time_) > 5*1e9 )
        {
            err_handler_(std::string("{\"code\":-2, \"desc\":\"串口打开错误\"}"));
            last_err_time_ = cur_time;
        }


        sleep(1);
        Init(uart_name, uart_baudrate);

        if(this->fd_ > 0)
        {
            // [1] create epoll object
            fd = epoll_create(1);

            // [2] add listen object
            struct epoll_event event;
            event.events = EPOLLIN;
            event.data.fd = fd_;

            if (epoll_ctl(fd, EPOLL_CTL_ADD, fd_, &event) < 0) {
                std::cerr << "Failed to add monitored object" << std::endl;
            }

            ::tcflush(fd_, TCIFLUSH);
        }

        if (try_to_quit_)
            break;
    }

    err_handler_(std::string("{\"code\":0, \"desc\":\"串口已经正确打开\"}"));

    unpack_cnt = 0;

    while (true) {
        // [3] wait event
        int n = epoll_wait(fd, events, 1, 0.7*1000);
        //std::cout << "epoll_wait ret:" << epoll_wait << std::endl;

        if (n < 0) {
            if (try_to_quit_)
                break;

            std::cerr << "Failed to wait response" << std::endl;
            continue;
        }

        is_pack = false;

        if( (n > 0) && (events[0].events & EPOLLIN) ) {
            //std::cout << "--------------" << std::endl;

            int len = -1;

            auto Read = [&] (uint8_t *data, int length) -> int {
                //return ::read(events[0].data.fd, data, length);
                len = ::read(events[0].data.fd, data, length);
                return len;
            };

            this->handler_(Read);

            //std::cout << "len: " << len << std::endl;

            if(len <= 0)
            {
                close(events[0].data.fd);
                this->fd_ = -1;
                close(fd);
                fd = -1;
                goto open_uart;
            }
        }
        
        if(GetCtlRun())
        {
            if(!is_pack)
            {
                Write(std::string("startlds$"));
                /*
                uint64_t cur_time = get_time();
                unpack_cnt += 1;
                if( ( (cur_time-last_err_time_) > 30*1e9 ) && (unpack_cnt > 40) )
                {
                    err_handler_(std::string("{\"code\":-3, \"desc\":\"长时间未收到点云数据\"}"));
                    last_err_time_ = cur_time;
                    unpack_cnt = 0;
                }
                */
                unpack_cnt += 1;
                if(unpack_cnt > 40)
                {
                    uint64_t cur_time = get_time();

                    if( ((cur_time/1e9) - last_pub_time_) > 30 )
                    {
                        if((cur_time-last_err_time_) > 30*1e9 ) 
                        {
                            err_handler_(std::string("{\"code\":-3, \"desc\":\"长时间未收到点云数据\"}"));
                            last_err_time_ = cur_time;
                            unpack_cnt = 0;
                        }
                    }
                }
            }
            else
            {
                unpack_cnt = 0;
            }
        }
        else
        {
            if(is_pack)
            {
                Write(std::string("holdlds$"));
                //std::cout << "send holdlds$ to lidar" << std::endl;
            }
        }
        
        if (try_to_quit_)
            break;
    }

    ::tcflush(fd_, TCIOFLUSH);
    quit_ = true;
}

#endif
