"""使用 O6 右手融合标定启动 MediaPipe 和单手 RViz。"""

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
                'pose_topic': '/right/mediapipe/hand_pose',
                'angles_topic': '/right/mediapipe/human_joint_angles',
                'debug_image_topic': '/right/mediapipe/debug_image',
                'target_joint_topic': '/right/linkerhand/target_joint_states',
                'status_topic': '/right/linkerhand/retargeting_status',
                'joint_states_topic': '/right/joint_states',
                'description_package': 'linkerhand_o6_right_description',
                'parameters_file': str(
                    package_share / 'config' / 'retargeting_o6_right.yaml'
                ),
            }.items(),
        )
    ])
