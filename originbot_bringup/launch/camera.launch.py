#!/usr/bin/python3

# Copyright (c) 2024, www.guyuehome.com
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_prefix('hobot_usb_cam'),
        "lib/hobot_usb_cam/config/usb_camera_calibration.yaml")
    print("config_file_path is ", config_file_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_camera_calibration_file_path',
            default_value=TextSubstitution(text=str(config_file_path)),
            description='camera calibration file path'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        
        Node(
            package='hobot_usb_cam',
            executable='hobot_usb_cam',
            name='hobot_usb_cam',
            parameters=[
                {"camera_calibration_file_path": LaunchConfiguration(
                    'usb_camera_calibration_file_path')},
                {"image_height": 480},
                {"image_width": 640},
                {"io_method":'mmap'},
                {"pixel_format": 'mjpeg2rgb'},
                {"video_device": '/dev/video8'},
                {"zero_copy":False}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
