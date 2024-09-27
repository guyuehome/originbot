#!/usr/bin/python3

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

def generate_launch_description():
    return LaunchDescription([
        Node(
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
                # {"zero_copy": False}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                    {"channel": 1},
                    {"in_mode": "shared_mem"},
                    {"in_format": "jpeg"},
                    {"out_mode": "ros"},
                    {"out_format": "bgr8"},
                    {"sub_topic": "/hbmem_img"},
                    {"pub_topic": "/image_raw"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        Node(
            package='originbot_demo',
            executable='transport_img',
            arguments=['--ros-args', '--log-level', 'error']
        ),
    ])