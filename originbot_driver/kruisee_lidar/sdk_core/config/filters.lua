--
-- Copyright (c) 2022 ECOVACS
--
-- Use of this source code is governed by a MIT-style
-- license that can be found in the LICENSE file or at
-- https://opensource.org/licenses/MIT
--

-- 该文件负责滤波器相关配置

-- 离群点滤波器
OUTLIER_FILTER = {
    -- 滤波半径
    radius = 0.15,
    -- 最小临近点集数
    min_count = 4,

    -- 滤波最大范围(5m)
    max_distance = 5
}

-- 近距离离群点滤波器
OUTLIER_FILTER_NEAR = {
    -- 滤波半径
    radius = 0.015,
    -- 最小临近点集数
    min_count = 3,

    -- 滤波最大范围(m)
    max_distance = 0.3,

    --正前方正常滤波的角度
    front_angle = 170,
    -- 两侧需要强滤波的角度
    side_angle = 60,
    -- 雷达缺口点逆时针旋转到正前方的角度
    lidar_angle = 213,
    -- 开启侧面角度滤波
    angle = true,
}

-- 平滑滤波器
SMOOTH_FILTER = {
    -- 平滑等级(滑动滤波窗口大小 = 2 * smooth_level + 1)
    level = 1,
    -- 平滑滤波误差(小于该值进行滤波)
    err = 0.1
}

-- 拖尾滤波器
TRAILING_FILTER = {
    -- 滤波窗口大小
    window = 5,

    -- 临近点数量
    neighbors = 3,

    -- 拖尾特征角度范围
    min_angle = 10,
    max_angle = 170
}

-- 角度滤波器(剔除指定角度范围内所有数据),支持不连续角度范围
ANGLE_FILTER = {
    -- 描述指定范围的起止角度值(角度范围0-360, 不能出现负向角度, 结束角度必须大于起始角度)
    range = { }
}

FILTERS = {
    outlier_filter = OUTLIER_FILTER,
    outlier_filter_near = OUTLIER_FILTER_NEAR,
    smooth_filter = SMOOTH_FILTER,
    angle_filter = ANGLE_FILTER,
    trailing_filter = TRAILING_FILTER,

    use_outlier_filter = false,
    use_outlier_filter_near = false,
    use_smooth_filter = false,
    use_angle_filter = false,
    use_trailing_filter = false,
}