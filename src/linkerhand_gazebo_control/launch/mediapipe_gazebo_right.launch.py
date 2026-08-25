"""启动右手 MediaPipe 到 Gazebo 的完整位置反馈同步。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_share = Path(
        get_package_share_directory('linkerhand_gazebo_control')
    )
    retargeting_share = Path(
        get_package_share_directory('linkerhand_retargeting')
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(gazebo_share / 'launch' / 'mediapipe_gazebo.launch.py')
            ),
            launch_arguments={
                'side': 'right',
                'pose_topic': '/right/mediapipe/hand_pose',
                'angles_topic': '/right/mediapipe/human_joint_angles',
                'debug_image_topic': '/right/mediapipe/debug_image',
                'target_joint_topic': '/right/linkerhand/target_joint_states',
                'status_topic': '/right/linkerhand/retargeting_status',
                'parameters_file': str(
                    retargeting_share / 'config' / 'retargeting_right.yaml'
                ),
            }.items(),
        )
    ])
