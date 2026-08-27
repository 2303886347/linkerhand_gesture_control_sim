"""启动 O6 左手 MediaPipe 到 Gazebo 的完整位置反馈同步。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_share = Path(
        get_package_share_directory('linkerhand_gazebo_control')
    )
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / 'launch' / 'mediapipe_gazebo.launch.py')
        ),
        launch_arguments={
            'model_id': 'o6',
            'side': 'left',
            'pose_topic': '/left/mediapipe/hand_pose',
            'angles_topic': '/left/mediapipe/human_joint_angles',
            'debug_image_topic': '/left/mediapipe/debug_image',
            'target_joint_topic': '/left/linkerhand/target_joint_states',
            'status_topic': '/left/linkerhand/retargeting_status',
        }.items(),
    )])
