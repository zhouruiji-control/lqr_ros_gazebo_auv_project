import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    auv_gazebo_path = get_package_share_directory('auv_gazebo')

    simple_auv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                auv_gazebo_path,
                'launch',
                'simple_auv.launch.py'
            )
        )
    )

    auv_pid_node = Node(
        package='auv_pid_ros',
        executable='auv_pid_node',
        name='auv_pid_node_visual',
        output='screen'
    )

    gazebo_pose_sync_node = Node(
        package='auv_pid_ros',
        executable='gazebo_pose_sync_node',
        name='gazebo_pose_sync_node',
        output='screen',
        remappings=[
            ('/current_depth', '/auv/current_trueposition')
        ]
    )

    delayed_control_nodes = TimerAction(
        period=6.0,
        actions=[
            auv_pid_node,
            gazebo_pose_sync_node
        ]
    )

    return LaunchDescription([
        simple_auv_launch,
        delayed_control_nodes
    ])
