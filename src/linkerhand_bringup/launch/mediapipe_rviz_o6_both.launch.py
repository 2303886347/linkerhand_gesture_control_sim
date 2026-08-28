"""快捷启动 O6 左手与 O6 右手。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('linkerhand_retargeting'))
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'mediapipe_rviz_both.launch.py')
        ),
        launch_arguments={
            'left_model': 'o6',
            'right_model': 'o6',
        }.items(),
    )])
