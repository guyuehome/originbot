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
import launch
import launch_ros
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'originbot_navigation'
    ld =  launch.LaunchDescription()
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name) 
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

      # Start robot localization using an Extended Kalman filter
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])

    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to disable use_sim_time'))
    ld.add_action(robot_localization_node)
    return ld
