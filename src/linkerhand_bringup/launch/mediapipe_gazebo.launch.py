"""Linker Hand L30/O6 单手 Gazebo 的参数式核心入口。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('linkerhand_gazebo_control'))
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'mediapipe_gazebo.launch.py')
        )
    )])
