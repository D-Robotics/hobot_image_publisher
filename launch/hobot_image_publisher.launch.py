from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        # 启动图片发布pkg
        Node(
            package='hobot_image_publisher',
            executable='hobot_image_pub',
            output='screen',
            parameters=[
                {"image_source": "lib/hobot_image_publisher/config/test1.nv12"},
                {"image_format": "nv12"},
                {"msg_pub_topic_name": "/test_msg"},
                {"output_image_w": 960},
                {"output_image_h": 544},
                {"source_image_w": 960},
                {"source_image_h": 544},
                {"fps": 5}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
