"""串联摄像头、MediaPipe、重定向和单手 Gazebo 控制。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration

from linkerhand_gazebo_control.bringup import resolve_gazebo_topics
from linkerhand_retargeting.bringup import resolve_retargeting_spec


def _launch_setup(context):
    mediapipe_share = Path(
        get_package_share_directory('mediapipe_hand_pose')
    )
    retargeting_share = Path(
        get_package_share_directory('linkerhand_retargeting')
    )
    gazebo_share = Path(
        get_package_share_directory('linkerhand_gazebo_control')
    )
    spec = resolve_retargeting_spec(
        LaunchConfiguration('model_id').perform(context),
        LaunchConfiguration('side').perform(context),
        retargeting_share,
        LaunchConfiguration('parameters_file').perform(context),
    )
    topics = resolve_gazebo_topics(
        spec.side,
        LaunchConfiguration('pose_topic').perform(context),
        LaunchConfiguration('angles_topic').perform(context),
        LaunchConfiguration('debug_image_topic').perform(context),
        LaunchConfiguration('target_joint_topic').perform(context),
        LaunchConfiguration('status_topic').perform(context),
    )
    if spec.uses_profile_defaults:
        get_logger('linkerhand_mediapipe_gazebo').warning(
            f'未找到 {spec.model_id}/{spec.side} 的默认标定 YAML；'
            '将回退到型号 profile 默认映射。可通过 '
            'parameters_file:=<yaml> 显式指定。'
        )

    parameters_file = (
        '' if spec.parameters_file is None else str(spec.parameters_file)
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
                    'pose_topic': topics.pose,
                    'angles_topic': topics.angles,
                    'debug_image_topic': topics.debug_image,
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
                    'parameters_file': parameters_file,
                    'model_id': spec.model_id,
                    'model_side': spec.side,
                    'input_topic': topics.pose,
                    'output_topic': topics.target_joint,
                    'status_topic': topics.status,
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
                    'model_id': spec.model_id,
                    'headless': LaunchConfiguration('headless'),
                    'trajectory_duration': LaunchConfiguration(
                        'trajectory_duration'
                    ),
                }.items(),
            )
        ],
    )

    return [
        LogInfo(
            msg=f'单手 Gazebo：model={spec.model_id}, side={spec.side}'
        ),
        perception,
        retargeting,
        gazebo,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left'),
        DeclareLaunchArgument('model_id', default_value='l30'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('processing_fps', default_value='15.0'),
        DeclareLaunchArgument('pose_topic', default_value=''),
        DeclareLaunchArgument(
            'angles_topic', default_value=''
        ),
        DeclareLaunchArgument(
            'debug_image_topic', default_value=''
        ),
        DeclareLaunchArgument(
            'target_joint_topic',
            default_value='',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='',
        ),
        DeclareLaunchArgument(
            'parameters_file',
            default_value='',
            description='个人标定 YAML；为空时按型号和手侧自动选择。',
        ),
        DeclareLaunchArgument('mediapipe_show_preview', default_value='true'),
        DeclareLaunchArgument('mirror_preview', default_value='true'),
        DeclareLaunchArgument('use_one_euro_filter', default_value='true'),
        DeclareLaunchArgument('one_euro_min_cutoff', default_value='0.8'),
        DeclareLaunchArgument('one_euro_beta', default_value='0.3'),
        DeclareLaunchArgument('trajectory_duration', default_value='0.15'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
