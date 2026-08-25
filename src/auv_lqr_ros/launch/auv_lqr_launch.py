import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==================================================
    # Gazebo launch
    # ==================================================

    auvGazeboPath = get_package_share_directory(
        'auv_gazebo'
    )

    gazeboLaunchFile = os.path.join(
        auvGazeboPath,
        'launch',
        'simple_auv.launch.py'
    )

    gazeboLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazeboLaunchFile
        )
    )


    # ==================================================
    # 反馈 + 噪声 + 滤波
    # ==================================================

    feedbackNode = Node(
        package='auv_lqr_ros',
        executable='force_depth_feedback_node',
        name='force_depth_feedback_node',
        output='screen',

        parameters=[
            {
                'entity_name': 'simple_auv',

                'reference_frame': 'world',

                'depth_noise_amplitude': 0.03,

                'velocity_noise_amplitude': 0.01,

                'filter_coefficient': 0.9
            }
        ]
    )


    # ==================================================
    # Gazebo力节点
    # ==================================================

    forceNode = Node(
        package='auv_lqr_ros',
        executable='gazebo_force_node',
        name='gazebo_force_node',
        output='screen',

        parameters=[
            {
                'force_scale': 1.0,

                'damping_coefficient': 5.0,

                'maximum_force': 100.0
            }
        ]
    )


    # ==================================================
    # LQR
    # ==================================================

    lqrNode = Node(
        package='auv_lqr_ros',
        executable='lqr_controller_node',
        name='lqr_controller_node',
        output='screen',

        parameters=[
            {
                'target_depth': 20.0,

                'csv_file':
                    'auv_lqr_15m.csv'
            }
        ]
    )


    # ==================================================
    # 延时启动
    # ==================================================

    delayedFeedback = TimerAction(
        period=6.0,
        actions=[
            feedbackNode
        ]
    )

    delayedForce = TimerAction(
        period=7.0,
        actions=[
            forceNode
        ]
    )

    delayedLqr = TimerAction(
        period=8.0,
        actions=[
            lqrNode
        ]
    )


    return LaunchDescription([
        gazeboLaunch,
        delayedFeedback,
        delayedForce,
        delayedLqr
    ])