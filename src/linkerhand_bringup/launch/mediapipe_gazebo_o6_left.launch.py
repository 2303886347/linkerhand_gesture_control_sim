"""正式入口：O6 左手 MediaPipe 到 Gazebo。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('linkerhand_gazebo_control'))
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'mediapipe_gazebo_o6_left.launch.py')
        )
    )])
