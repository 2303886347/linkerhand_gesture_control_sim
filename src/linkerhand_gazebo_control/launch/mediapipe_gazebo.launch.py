"""串联摄像头、MediaPipe、重定向和单手 Gazebo 控制。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mediapipe_share = Path(
        get_package_share_directory('mediapipe_hand_pose')
    )
    retargeting_share = Path(
        get_package_share_directory('linkerhand_retargeting')
    )
    gazebo_share = Path(
        get_package_share_directory('linkerhand_gazebo_control')
    )

    perception = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(mediapipe_share / 'launch' / 'pipeline.launch.py')
                ),
                launch_arguments={
                    'device': LaunchConfiguration('device'),
                    'width': LaunchConfiguration('width'),
                    'height': LaunchConfiguration('height'),
                    'camera_fps': LaunchConfiguration('camera_fps'),
                    'processing_fps': LaunchConfiguration('processing_fps'),
                    'target_hand': LaunchConfiguration('side'),
                    'pose_topic': LaunchConfiguration('pose_topic'),
                    'angles_topic': LaunchConfiguration('angles_topic'),
                    'debug_image_topic': LaunchConfiguration(
                        'debug_image_topic'
                    ),
                    'camera_show_preview': 'false',
                    'mediapipe_show_preview': LaunchConfiguration(
                        'mediapipe_show_preview'
                    ),
                    'mirror_preview': LaunchConfiguration('mirror_preview'),
                    'use_one_euro_filter': LaunchConfiguration(
                        'use_one_euro_filter'
                    ),
                    'one_euro_min_cutoff': LaunchConfiguration(
                        'one_euro_min_cutoff'
                    ),
                    'one_euro_beta': LaunchConfiguration('one_euro_beta'),
                }.items(),
            )
        ],
    )

    retargeting = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        retargeting_share
                        / 'launch'
                        / 'retargeting.launch.py'
                    )
                ),
                launch_arguments={
                    'parameters_file': LaunchConfiguration(
                        'parameters_file'
                    ),
                    'input_topic': LaunchConfiguration('pose_topic'),
                    'output_topic': LaunchConfiguration('target_joint_topic'),
                    'status_topic': LaunchConfiguration('status_topic'),
                    'use_rviz_adapter': 'false',
                }.items(),
            )
        ],
    )

    gazebo = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(gazebo_share / 'launch' / 'gazebo_control.launch.py')
                ),
                launch_arguments={
                    'side': LaunchConfiguration('side'),
                    'headless': LaunchConfiguration('headless'),
                    'trajectory_duration': LaunchConfiguration(
                        'trajectory_duration'
                    ),
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('processing_fps', default_value='15.0'),
        DeclareLaunchArgument('pose_topic', default_value='/left/mediapipe/hand_pose'),
        DeclareLaunchArgument(
            'angles_topic', default_value='/left/mediapipe/human_joint_angles'
        ),
        DeclareLaunchArgument(
            'debug_image_topic', default_value='/left/mediapipe/debug_image'
        ),
        DeclareLaunchArgument(
            'target_joint_topic',
            default_value='/left/linkerhand/target_joint_states',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/left/linkerhand/retargeting_status',
        ),
        DeclareLaunchArgument(
            'parameters_file',
            default_value=str(
                retargeting_share / 'config' / 'retargeting_left.yaml'
            ),
        ),
        DeclareLaunchArgument('mediapipe_show_preview', default_value='true'),
        DeclareLaunchArgument('mirror_preview', default_value='true'),
        DeclareLaunchArgument('use_one_euro_filter', default_value='true'),
        DeclareLaunchArgument('one_euro_min_cutoff', default_value='0.8'),
        DeclareLaunchArgument('one_euro_beta', default_value='0.3'),
        DeclareLaunchArgument('trajectory_duration', default_value='0.15'),
        DeclareLaunchArgument('headless', default_value='false'),
        perception,
        retargeting,
        gazebo,
    ])
