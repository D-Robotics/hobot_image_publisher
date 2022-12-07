from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        # 启动视频发布pkg
        Node(
            package='hobot_image_publisher',
            executable='hobot_image_pub',
            output='screen',
            parameters=[
                {"image_source": "./config/test1.h264"},
                {"image_format": "h264"},
                {"msg_pub_topic_name": "/test_h264"},
                {"fps": 30},
                {"is_loop": True},
                {"is_shared_mem": True}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
