#!/usr/bin/python3
# Copyright 2020, EAIBOT
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

    driver_node = LifecycleNode(package='kruisee_lidar',
                                executable='kruisee_lidar_node',
                                name='kruisee_lidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[
                                  {'sdk_core_path':"install/kruisee_lidar"},
                                  {'topic': "scan"},
                                  {'queue_size':50},
                                  {'frame_id':"laser_link"},
                                  {'use_polar_coordinates': True},
                                  {'use_cartesian_cooridnates':False},
                                ],
                                namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser1',
                    arguments=['0', '0', '0.18','0', '0', '0', '1','/base_link','/laser_link'],
                    )
    tf2_node1 = Node(package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub_laser2',
                arguments=['0', '0', '0.02','0', '0', '0', '1','base_footprint','base_link'],
                )
    tf2_node2 = Node(package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub_laser3',
                arguments=['0', '0', '0.02','0', '0', '0', '1','base_footprint','odom'],
                )
    tf2_node3 = Node(package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub_laser4',
                arguments=['0', '0', '0.02','0', '0', '0', '1','odom','base_footprint'],
                )   

    return LaunchDescription([
        driver_node,
        tf2_node,
        tf2_node1,
        tf2_node2,
        tf2_node3
    ])
