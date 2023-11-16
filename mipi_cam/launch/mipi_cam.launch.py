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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mipi_camera_calibration_file_path',
            default_value='/userdata/dev_ws/src/origineye/mipi_cam/config/SC132GS_calibration.yaml',
            description='mipi camera calibration file path'),
        DeclareLaunchArgument(
            'mipi_out_format',
            default_value='nv12',
            description='mipi camera out format'),
        DeclareLaunchArgument(
            'mipi_image_width',
            default_value='1088',
            description='mipi camera out image width'),
        DeclareLaunchArgument(
            'mipi_image_height',
            default_value='1280',
            description='mipi camera out image height'),
        DeclareLaunchArgument(
            'mipi_io_method',
            default_value='shared_mem',
            description='mipi camera out io_method'),
        DeclareLaunchArgument(
            'mipi_video_device',
            default_value='F37',
            description='mipi camera device'),
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"camera_calibration_file_path": LaunchConfiguration(
                    'mipi_camera_calibration_file_path')},
                {"out_format": LaunchConfiguration('mipi_out_format')},
                {"image_width": LaunchConfiguration('mipi_image_width')},
                {"image_height": LaunchConfiguration('mipi_image_height')},
                {"io_method": LaunchConfiguration('mipi_io_method')},
                {"video_device": LaunchConfiguration('mipi_video_device')},
                {"rotate_degree": 90},
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
