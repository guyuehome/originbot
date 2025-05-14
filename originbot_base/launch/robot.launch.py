from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyS1',
                                          description='usb bus name, e.g. ttyS1')

    correct_factor_vx_arg = DeclareLaunchArgument('correct_factor_vx', default_value='0.898',
                                                  description='correct factor vx, e.g. 0.9')

    correct_factor_vth_arg = DeclareLaunchArgument('correct_factor_vth', default_value='0.874',
                                                   description='correct factor vth, e.g. 0.9')

    auto_stop_on_arg = DeclareLaunchArgument('auto_stop_on', default_value='true',
                                             description='auto stop if no cmd received, true or true')

    use_imu_arg = DeclareLaunchArgument('use_imu', default_value='false',
                                        description='if has imu sensor to drive')

    pub_odom_arg = DeclareLaunchArgument('pub_odom', default_value='true',
                                         description='publish odom to base_footprint tf, true or true')

    originbot_base_node = Node(
        package='originbot_base',
        executable='originbot_base',
        output='screen',
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'correct_factor_vx': LaunchConfiguration('correct_factor_vx'),
            'correct_factor_vth': LaunchConfiguration('correct_factor_vth'),
            'auto_stop_on': LaunchConfiguration('auto_stop_on'),
            'use_imu': LaunchConfiguration('use_imu'),
            'pub_odom': LaunchConfiguration('pub_odom'),
        }]
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.05325", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "base_footprint", "--child-frame-id", "base_link"]
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "base_link", "--child-frame-id", "imu_link"]
    )

    return LaunchDescription([
        port_name_arg,
        correct_factor_vx_arg,
        correct_factor_vth_arg,
        auto_stop_on_arg,
        use_imu_arg,
        pub_odom_arg,
        originbot_base_node,
        base_footprint_tf,
        imu_tf
    ])