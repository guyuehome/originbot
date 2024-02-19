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
    mipi_cam_device_arg = DeclareLaunchArgument(
        'device',
        default_value='GC4663',
        description='mipi camera device')

    mipi_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mipi_cam'),
                'launch/mipi_cam.launch.py')),
        launch_arguments={
            'mipi_image_width': '960',
            'mipi_image_height': '544',
            'mipi_io_method': 'shared_mem',
            'mipi_video_device': LaunchConfiguration('device')
        }.items()
    )

    # nv12->jpeg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image_jpeg'
        }.items()
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
    websocket_node = Node(
        package='websocket',
        executable='websocket',
        output='screen',
        parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"only_show_image": True}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    image_transport_node = Node(
        package="utils",
        executable="image_transport_node"
        )
    
    vlpr_node = Node(
        package="vlpr",
        executable="vlpr_node"
        )
    return LaunchDescription([
        mipi_cam_device_arg,
        mipi_node,
        # jpeg_codec_node,
        image_transport_node,
        hobot_codec_node,
        websocket_node,
        vlpr_node
    ])