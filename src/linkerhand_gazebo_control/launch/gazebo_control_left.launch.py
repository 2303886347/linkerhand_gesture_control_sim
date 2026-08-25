"""启动左手 Gazebo 运动学位置反馈同步。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('linkerhand_gazebo_control')
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_share / 'launch' / 'gazebo_control.launch.py')
            ),
            launch_arguments={'side': 'left'}.items(),
        )
    ])
