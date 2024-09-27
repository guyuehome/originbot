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

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    usb_cam = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='hobot_usb_cam',
        parameters=[
            {"camera_calibration_file_path": "/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml"},
            {"frame_id": "default_usb_cam"},
            {"framerate": 30},
            {'image_width': 640},
            {'image_height': 480},
            {"io_method": "mmap"},
            {"pixel_format": "mjpeg"},
            {"video_device": "/dev/video8"},
            {"zero_copy": True}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # mjpeg->bgr8
    bgr8codec = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
                {"channel": 1},
                {"in_mode": "shared_mem"},
                {"in_format": "jpeg"},
                {"sub_topic": "/hbmem_img"},
                {"out_mode": "ros"},
                {"out_format": "bgr8"},
                {"pub_topic": "/image_raw"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    vlpr_node = Node(
        package="originbot_vlpr",
        executable="vlpr_node",
        remappings=[("/vlpr_node/image_sub","/image_raw")],
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
            {"sub_topic": "/vlp_image"},
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
        bgr8codec,
        vlpr_node,
        hobot_codec_node,
        web_node
    ])