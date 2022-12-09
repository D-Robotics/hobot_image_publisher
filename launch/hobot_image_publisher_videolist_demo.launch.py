import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/hobot_websocket_service.launch.py'))
    )

    return LaunchDescription([
        #启动webservice
        web_service_launch_include,
        # 启动图片发布pkg
        Node(
            package='hobot_image_publisher',
            executable='hobot_image_pub',
            output='screen',
            parameters=[
                # image_source为本地视频文件的路径或保存本地视频文件路径的list文件，具体使用方法可参考README
                {"image_source": "./config/video.list"},
                # image_format为支持发布的本地视频格式，当前支持的格式有h264/h265/mp4
                {"image_format": "mp4"},
                {"msg_pub_topic_name": "/hbmem_img"},
                {"fps": 30},# 优先使用从本地视频文件中解析出来的帧率，解析失败时会采用此fps
                {"is_loop": True},# 是否循环发布  True/False
                {"is_shared_mem": True}# 是否使用share_mem的方式通信, True/False
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动nv12图片解码&发布pkg
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                #处理通道号 取值0~3
                {"channel": 1},
                #接入数据传输的方式 ros/shared_mem
                {"in_mode": "shared_mem"},
                #订阅的数据格式, 其中h264/h265为视频回灌会订阅到的格式
                {"in_format": "h264"},
                {"out_mode": "ros"},
                # 解码后的图片格式，订阅h264/h265格式，解码后的格式只能为bgr8/rgb8/nv12
                {"out_format": "nv12"},
                {"sub_topic": "/hbmem_img"},
                {"pub_topic": "/image_nv12"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动jpeg图片编码&发布pkg
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": 1},
                {"in_mode": "ros"},
                {"in_format": "nv12"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"sub_topic": "/image_nv12"},
                {"pub_topic": "/image_jpeg"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动web展示pkg
        Node(
            package='websocket',
            executable='websocket',
            output='screen',
            parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"only_show_image": True},
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
