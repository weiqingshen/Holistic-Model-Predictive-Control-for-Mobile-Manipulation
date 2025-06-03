from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动发布者节点
        Node(
            package='my_pose_project',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen',
        ),
        # 启动订阅者节点
        Node(
            package='my_pose_project',
            executable='communication',
            name='communication',
            output='screen',
        ),
    ])

