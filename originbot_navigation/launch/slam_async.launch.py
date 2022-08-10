from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('originbot_navigation')

    slam_config = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'config', 'slam_async.yaml'])

    slam = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
              slam_config,
              {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam)
    return ld
