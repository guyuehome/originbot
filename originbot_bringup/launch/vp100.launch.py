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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

def generate_launch_description():
    share_dir = get_package_share_directory('vp100_ros2')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'vp100_ros2_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'vp100.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='vp100_ros2',
                                executable='vp100_ros2_node',  # Changed from node_executable to executable
                                name='vp100_ros2_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',  # Changed from node_executable to executable
                    name='static_tf_pub_laser',  # Changed from node_name to name
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_link'],
                    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
    ])