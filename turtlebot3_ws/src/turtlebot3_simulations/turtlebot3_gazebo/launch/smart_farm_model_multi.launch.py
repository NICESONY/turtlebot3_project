#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    # ------------ Args ------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ------------ Paths ------------
    pkg_tb3_gz  = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo  = get_package_share_directory('gazebo_ros')
    world = os.path.join(pkg_tb3_gz, 'worlds', 'smart_farm_fianl.world')

    # ------------ Gazebo (global, once) ------------
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py'))
    )

    # ------------ Robot poses ------------
    initial_poses = {
        'robot1': {'x': '0.0', 'y': '0.0', 'yaw': '0.0'},
        'robot2': {'x': '2.0', 'y': '1.0', 'yaw': '1.57'},
    }

    # ------------ Namespaced robot groups ------------
    groups = []
    delay = 0.0  # spawn 간격 (서비스 준비시간 확보용, 필요 없으면 0으로)
    for ns, p in initial_poses.items():

        # 1) robot_state_publisher (네임스페이스/TF prefix)
        rsp_ld = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gz, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # robot_state_publisher.launch.py가 frame_prefix 지원할 때만 사용
                # 'frame_prefix': ns + '/'
            }.items()
        )

        # 2) Gazebo에 직접 spawn (spawn_entity.py)
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{ns}',
            output='screen',
            arguments=[
                '-entity', ns,                     # ★ 모델 이름 고유!
                '-topic',  'robot_description',    # ns 안에서 올라온 robot_description 사용
                '-x', p['x'], '-y', p['y'], '-Y', p['yaw']
            ],
        )

        groups.append(
            GroupAction([
                PushRosNamespace(ns),
                rsp_ld,
                TimerAction(period=delay, actions=[spawn_node]),
            ])
        )
        delay += 2.0

    # ------------ LaunchDescription ------------
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gzserver,
        gzclient,
        *groups
    ])
    return ld
