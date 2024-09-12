from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([        
        # 启动qrcode_control节点
        Node(
            package='originbot_qrcode_detect',
            executable='qrcode_control',
            name='qrcode_control'
        ),

        # 启动qr_decoder节点  
        Node(
            package='originbot_qrcode_detect',
            executable='qr_decoder',
            name='qr_decoder'
        )
    ])