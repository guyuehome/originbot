#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

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
    usb_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('originbot_bringup'),
                    'launch/camera_internal.launch.py')),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480'
        }.items()
    )
    image_transport_node = Node(
        package='utils',
        executable='image_transport_node',
        arguments=['--ros-args', '--log-level', 'error']
    )
    qr_detection_node = Node(
        package='qr_code_detection',
        executable='qr_detection_node',
        remappings=[("/qrcode_detected/image_sub","/image_out/compressed")],
        arguments=['--ros-args', '--log-level', 'info']
    )
    hobot_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
                {"channel": 1},
                {"in_mode": "ros"},
                {"in_format": "bgr8"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"sub_topic": "/qrcode_detected/img_result"},
                {"pub_topic": "/image_jpeg"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    # web
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_only_show_image': 'True'
        }.items()
    )
    return LaunchDescription([
        usb_cam,
        image_transport_node,
        qr_detection_node,
        hobot_codec_node,
        web_node
    ])
