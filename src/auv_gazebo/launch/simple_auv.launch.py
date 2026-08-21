import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取功能包安装路径
    auv_description_path = get_package_share_directory(
        "auv_description"
    )

    auv_gazebo_path = get_package_share_directory(
        "auv_gazebo"
    )

    # AUV 模型文件
    xacro_file = os.path.join(
        auv_description_path,
        "urdf",
        "simple_auv.urdf.xacro"
    )

    # Gazebo 世界文件
    world_file = os.path.join(
        auv_gazebo_path,
        "worlds",
        "empty_underwater.world"
    )

    # 将 xacro 转换为 robot_description
    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file
        ])
    }

    # 启动 Gazebo
    # init：ROS2 初始化插件
    # factory：提供 /spawn_entity
    # force_system：提供外力相关服务
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_file,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            "-s",
            "libgazebo_ros_force_system.so"
        ],
        output="screen"
    )

    # 发布机器人模型描述
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # 把 simple_auv 生成到 Gazebo
    spawn_auv = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_simple_auv",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "simple_auv",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0"
        ]
    )

    # 等 Gazebo 启动后再生成模型
    delayed_spawn_auv = TimerAction(
        period=5.0,
        actions=[spawn_auv]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn_auv
    ])