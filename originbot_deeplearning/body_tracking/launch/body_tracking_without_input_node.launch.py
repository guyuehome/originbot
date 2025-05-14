# Copyright (c) 2024，D-Robotics.
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
    # 人体跟随
    body_tracking_node = Node(
        package='body_tracking',
        executable='body_tracking',
        output='screen',
        parameters=[
            {"ai_msg_sub_topic_name": "/hobot_mono2d_body_detection"},
            {"activate_wakeup_gesture": 0},
            {"img_width": 960},
            {"img_height": 544},
            {"track_serial_lost_num_thr": 30},
            {"linear_velocity": 0.2},
            {"angular_velocity": 0.4},
            {"activate_robot_move_thr": 5}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    mono2d_body_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mono2d_body_detection'),
                'launch/mono2d_body_detection_without_input_node.launch.py')),
        launch_arguments={
            'smart_topic': '/hobot_mono2d_body_detection',
            'mono2d_body_pub_topic': '/hobot_mono2d_body_detection'
        }.items()
    )

    return LaunchDescription([
        mono2d_body_det_node,
        body_tracking_node
    ])
