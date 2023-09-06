# Copyright (c) 2022，Horizon Robotics.
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

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # parking search
    parking_search_node = Node(
        package='parking_search',
        executable='parking_search',
        output='screen',
        parameters=[
            {"ai_msg_sub_topic_name": "/ai_msg_parking_perception"},
            {"twist_pub_topic_name": "/cmd_vel"},
            {"area_height": 40},
            {"area_width": 120},
            {"ingored_bottom": 40},
            {"mid_parking_iou": 0.7},
            {"sides_parking_iou": 0.6},
            {"mid_path_iou": 0.9},
            {"sides_parking_iou": 0.8},
            {"arrived_count": 400},
            {"move_step": 0.1},
            {"rotate_step": 0.1}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 车位区域寻找算法
    parking_perception_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('parking_perception'),
                'launch/parking_perception.launch.py')),
        launch_arguments={
            'parking_perception_pub_topic': '/ai_msg_parking_perception'
        }.items()
    )

    return LaunchDescription([
        parking_perception_node,
        parking_search_node
    ])
