import os
import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
    #                                          description='Use simulation clock if true')
    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyS3',
                                         description='usb bus name, e.g. ttyS3')
    
    originbot_base_node = Node(
        package='originbot_base',
        executable='originbot_base', 
        output='screen',
        emulate_tty=True,
        parameters=[{
                # 'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port_name': LaunchConfiguration('port_name'), 
        }])

    originbot_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        emulate_tty=True,
        arguments="0.0 0.0 0.0 0.0 0.0 0.0 /base_link /imu_link".split(
                ' '))
        
    return LaunchDescription([
        # use_sim_time_arg,
        port_name_arg,
        originbot_base_node,
        originbot_imu_tf
    ])
