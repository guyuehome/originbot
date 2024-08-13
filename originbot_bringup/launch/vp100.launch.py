#!/usr/bin/python3

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable


def generate_launch_description():
    vp100_launch = ExecuteProcess(
        cmd = [
            FindExecutable(name = "ros2"), # 不可以有空格
            " launch",
            " vp100_ros2",
            " vp100_launch.py"
        ],
        output="both",
        shell=True)

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments="0.0 0.0 0.1 0.0 0.0 0.0 /base_link /laser_link".split(' ')
    )

    return LaunchDescription([
        vp100_launch,
        tf2_node
    ])
