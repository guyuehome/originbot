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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket_service.launch.py'))
    )
    mipi_cam_node = Node(
        package='mipi_cam',
        executable='mipi_cam',
        output='screen',
        parameters=[
                {"camera_calibration_file_path": "/opt/tros/lib/mipi_cam/config/GC4663_calibration.yaml"},
                {"out_format": "nv12"},
                {"image_width": 640},
                {"image_height": 480},
                {"io_method": "shared_mem"},
                {"video_device": "GC4663"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
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
    websocket_node = Node(
        package='websocket',
        executable='websocket',
        output='screen',
        parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"only_show_image": True},
                {"smart_topic": "/ai_msg_mono2d_trash_detection"},
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    image_transport_node = Node(
        package='utils',
        executable='image_transport_node',
        arguments=['--ros-args', '--log-level', 'error']
    )
    qr_detection_node = Node(
        package='qr_code_detection',
        executable='qr_detection_node',
    )
    return LaunchDescription([
        web_service_launch_include,
        mipi_cam_node,
        image_transport_node,
        qr_detection_node,
        hobot_codec_node,
        websocket_node,
    ])
