import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    viewer = os.path.join(
        get_package_share_directory('soccer_game'), 'viewer', 'index.html')

    return LaunchDescription([
        DeclareLaunchArgument('open_ui', default_value='true',
                              description='브라우저 3D 뷰어 자동 열기'),
        # 심판: 헤드리스 물리 + WebSocket(8765) 방송. turtlesim 안 씀.
        Node(
            package='soccer_game',
            executable='soccer_referee',
            output='screen'),
        # three.js 뷰어를 기본 브라우저로 자동 오픈
        ExecuteProcess(
            cmd=['xdg-open', viewer],
            condition=IfCondition(LaunchConfiguration('open_ui')),
            output='screen'),
    ])
