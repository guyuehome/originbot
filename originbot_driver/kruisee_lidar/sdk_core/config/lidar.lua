--
-- Copyright (c) 2022 ECOVACS
--
-- Use of this source code is governed by a MIT-style
-- license that can be found in the LICENSE file or at
-- https://opensource.org/licenses/MIT
--

-- 该文件负责雷达相关属性配置

-- 雷达默认测量值(该参数仅由 kruisee 开发团队维护, 禁止用户进行修改)
LIDAR30M = 35
LIDAR50M = 55

LIDAR25M = 25

LIDAR = {
    -- 定义雷达测量范围
    max_range = LIDAR25M,

    -- 定义雷达零度位置(相对结构0度逆时针)
    zero_angle = 0,
}