
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # usb cam图片发布pkg
    usb_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('originbot_bringup') + '/launch/camera.launch.py')
    )

    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_sub_topic': '/hbmem_img',
            'codec_out_mode': 'ros',
            'codec_pub_topic': '/image_mjpeg'
        }.items()
    )

    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
        launch_arguments={
            'websocket_image_topic': '/image_mjpeg',
            'websocket_image_type': 'mjpeg',
            'websocket_only_show_image': 'True'
        }.items()
    )

    return LaunchDescription([
        usb_cam,
        # jpeg_codec_node,
        web_node
    ])
