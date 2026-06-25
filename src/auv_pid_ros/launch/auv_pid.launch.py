from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auv_pid_ros',
            executable='auv_pid_node',
            name='auv_pid_node',
            output='screen'
        )
    ])