--
-- Copyright (c) 2022 ECOVACS
--
-- Use of this source code is governed by a MIT-style
-- license that can be found in the LICENSE file or at
-- https://opensource.org/licenses/MIT
--

-- 该文件负责激光雷达通讯相关配置

-- 串口通讯方式配置
UART = {
    -- 串口号
    name = "/dev/ttyUSB0",
    -- 波特率
    baudrate = 460800,
}

-- 以太网通讯方式配置
ETHERNET = {
    -- 激光雷达IP
    radar_ip = "169.254.119.2",
    -- 激光雷达端口
    radar_port = 3000,

    -- PC监听端口
    listen_port = 2000,
}

PROTOCOL = {
    uart = UART,
    ethernet = ETHERNET,

    use_net_protocol = false,
    use_uart_protocol = false
}