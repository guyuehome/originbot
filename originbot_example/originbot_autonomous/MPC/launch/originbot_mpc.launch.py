import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #设置文件名
    pkg_name = 'originbot_autonomous'
    yaml_name = 'mpc_settings.yaml'

    #创建启动文件描述
    ld = LaunchDescription()
    #获取文件路径信息
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    yaml_path = os.path.join(pkg_share,'config',yaml_name)

    #启动节点
    democar_mpc_node = Node(
        package='originbot_autonomous',
        executable='mpc_node',
        name='mpc_controller',
        parameters=[yaml_path],
        output='screen',
    )

    ld.add_action(democar_mpc_node)

    return ld
