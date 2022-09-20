from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        # 启动图片发布pkg，output_image_w与output_image_h设置为0代表不改变图片的分辨率
        Node(
            package='hobot_image_publisher',
            executable='hobot_image_pub',
            output='screen',
            parameters=[
                {"image_source": "./config/test1.nv12"},
                {"image_format": "nv12"},
                {"msg_pub_topic_name": "/test_msg"},
                {"output_image_w": 0},
                {"output_image_h": 0},
                {"source_image_w": 960},
                {"source_image_h": 544},
                {"fps": 10},
                {"is_loop": True},
                {"is_shared_mem": True}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
