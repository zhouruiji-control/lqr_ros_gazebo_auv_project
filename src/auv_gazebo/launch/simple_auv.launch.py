import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    auv_description_path = get_package_share_directory('auv_description')
    auv_gazebo_path = get_package_share_directory('auv_gazebo')

    xacro_file = os.path.join(
        auv_description_path,
        'urdf',
        'simple_auv.urdf.xacro'
    )

    world_file = os.path.join(
        auv_gazebo_path,
        'worlds',
        'empty_underwater.world'
    )

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_auv = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_simple_auv',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_auv',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ]
    )

    delayed_spawn_auv = TimerAction(
        period=5.0,
        actions=[spawn_auv]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn_auv
    ])
