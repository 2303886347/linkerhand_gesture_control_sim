"""Linker Hand 多型号双手 RViz 的参数式核心入口。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    retargeting_share = Path(
        get_package_share_directory('linkerhand_retargeting')
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    retargeting_share
                    / 'launch'
                    / 'mediapipe_rviz_both.launch.py'
                )
            )
        )
    ])
