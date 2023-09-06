--
-- Copyright (c) 2022 ECOVACS
--
-- Use of this source code is governed by a MIT-style
-- license that can be found in the LICENSE file or at
-- https://opensource.org/licenses/MIT
--

include "lidar.lua"
include "protocol.lua"
include "filters.lua"

options = {
    -- 导入通信协议
    protocol = PROTOCOL,

    -- 导入滤波器集合
    filters = FILTERS,

    -- 导入雷达相关属性
    lidar = LIDAR,
}

-- [1] 选择通信方式，二选一将使用的注释掉或者改为false
PROTOCOL.use_net_protocol = false    -- 采用以太网通信(默认关闭)
PROTOCOL.use_uart_protocol = true  -- 采用串口通信(默认开启)

-- [2] 设置离群点滤波
OUTLIER_FILTER.radius = 0.04       -- 设置滤波半径
OUTLIER_FILTER.min_count = 3        -- 设置滤波点数，在滤波半径内点数小于该值将会该滤除
OUTLIER_FILTER.max_distance = 2     -- 设置半径滤波最大范围，单位m
FILTERS.use_outlier_filter = true   -- 开启半径滤波

-- 设置近距离离群点滤波
OUTLIER_FILTER_NEAR.radius = 0.015       -- 设置滤波半径
OUTLIER_FILTER_NEAR.min_count = 3        -- 设置滤波点数，在滤波半径内点数小于该值将会该滤除
OUTLIER_FILTER_NEAR.max_distance = 0.3   -- 设置半径滤波最大范围，单位m
FILTERS.use_outlier_filter_near = false   -- 开启近距离半径滤波(默认关闭)

OUTLIER_FILTER_NEAR.front_angle = 172    -- 正前方正常滤波的角度 86*2
OUTLIER_FILTER_NEAR.side_angle = 67      -- 两侧需要强滤波的角度
OUTLIER_FILTER_NEAR.lidar_angle = 213    -- 雷达缺口点逆时针旋转到正前方的角度
OUTLIER_FILTER_NEAR.angle = false         -- 开启侧面角度滤波(默认关闭)

-- [3] 设置拖尾滤波
TRAILING_FILTER.window = 5          -- 滤波窗口大小
TRAILING_FILTER.neighbors = 3       -- 临近点数
TRAILING_FILTER.min_angle = 10      -- 拖尾特征角度1
TRAILING_FILTER.max_angle = 170     -- 拖尾特征角度2
FILTERS.use_trailing_filter = true  -- 开启拖尾滤波

-- [3] 设置平滑滤波
SMOOTH_FILTER.level = 1             -- 设置平滑等级
SMOOTH_FILTER.err = 0.05            -- 设置平滑滤波误差，小于该值进行滤除
FILTERS.use_smooth_filter = false   -- 开启平滑滤波(默认关闭)

-- [4] 设置角度滤波
ANGLE_FILTER.range = { {90, 180} }  -- 设置角度滤波范围，角度范围相当于设置零度位置后的角度范围
FILTERS.use_angle_filter = false    -- 开启角度滤波(默认关闭)

-- [5] 设置雷达零度位置，相当于机械零度逆时针旋转指定角度
LIDAR.zero_angle = 0.0

-- [6] 设置雷达测距范围(该属性由硬件特性决定)
LIDAR.max_range = LIDAR25M

return options