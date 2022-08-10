import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='port_name',
                                             default_value='ttyS3'),
        launch.actions.DeclareLaunchArgument(name='correct_factor_vx',
                                             default_value='1.0'),
        launch.actions.DeclareLaunchArgument(name='correct_factor_vth',
                                             default_value='0.868'),                                                                                
        launch.actions.DeclareLaunchArgument(name='open_rviz',
                                             default_value='false'),
        launch_ros.actions.Node(
            package='rviz2',
            name='rviz2',
            executable='rviz2',
            on_exit=launch.actions.Shutdown(),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('open_rviz'))),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('originbot_base'),
                             'launch/robot.launch.py')),
            launch_arguments={
                'port_name': launch.substitutions.LaunchConfiguration('port_name'),
                'correct_factor_vx': launch.substitutions.LaunchConfiguration('correct_factor_vx'),
                'correct_factor_vth': launch.substitutions.LaunchConfiguration('correct_factor_vth'),
            }.items())
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
