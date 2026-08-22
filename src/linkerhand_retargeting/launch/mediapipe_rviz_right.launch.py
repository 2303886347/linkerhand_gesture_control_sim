"""使用右手标定启动 MediaPipe、角度转换和右手 RViz。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


PACKAGE_NAME = 'linkerhand_retargeting'


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_share / 'launch' / 'mediapipe_rviz.launch.py')
            ),
            launch_arguments={
                'target_hand': 'right',
                'description_package': 'linkerhand_l30_right_description',
                'parameters_file': str(
                    package_share / 'config' / 'retargeting_right.yaml'
                ),
            }.items(),
        )
    ])
