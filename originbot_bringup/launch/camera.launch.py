from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"out_format": "nv12"},
                {"image_width": 480},
                {"image_height": 272},
                {"io_method": "shared_mem"},
                {"video_device": "GC4663"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动nv12->jpeg图片编码&发布pkg
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": 1},
                {"in_mode": "shared_mem"},
                {"in_format": "nv12"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"sub_topic": "/hbmem_img"},
                {"pub_topic": "/image_jpeg"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动jpeg->bgr8图片解码&发布pkg
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": 1},
                {"in_mode": "ros"},
                {"in_format": "jpeg"},
                {"out_mode": "ros"},
                {"out_format": "bgr8"},
                {"sub_topic": "/image_jpeg"},
                {"pub_topic": "/image_bgr8"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
    ])