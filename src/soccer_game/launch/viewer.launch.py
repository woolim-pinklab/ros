"""뷰어만 띄운다 — 심판은 다른 머신에서 이미 돌고 있을 때 쓴다.

심판(물리 계산)은 한 머신에서 한 명만 띄우고, 나머지는 이걸로 화면만 본다.

    ros2 launch soccer_game viewer.launch.py host:=192.168.0.10
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    viewer = os.path.join(
        get_package_share_directory('soccer_game'), 'viewer', 'index.html')

    return LaunchDescription([
        DeclareLaunchArgument(
            'host', default_value='localhost',
            description='심판이 도는 머신의 IP (기본: 이 머신)'),
        DeclareLaunchArgument(
            'port', default_value='8765',
            description='심판 WebSocket 포트'),
        ExecuteProcess(
            cmd=['xdg-open',
                 ['file://', viewer,
                  '?host=', LaunchConfiguration('host'),
                  '&port=', LaunchConfiguration('port')]],
            output='screen'),
    ])
