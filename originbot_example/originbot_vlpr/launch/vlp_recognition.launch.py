from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    image_transport_node = Node(
        package="utils",
        executable="image_transport_node"
        )
    vlpr_node = Node(
        package="originbot_vlpr",
        executable="vlpr_node"
        )
    launch_description = LaunchDescription([image_transport_node,vlpr_node])

    return launch_description