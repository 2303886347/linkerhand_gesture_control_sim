"""Linker Hand L30/O6 参数式单手 RViz 正式入口。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from linkerhand_retargeting.bringup import resolve_retargeting_spec


def _launch_setup(context):
    retargeting_share = Path(
        get_package_share_directory('linkerhand_retargeting')
    )
    spec = resolve_retargeting_spec(
        LaunchConfiguration('model_id').perform(context),
        LaunchConfiguration('side').perform(context),
        retargeting_share,
        LaunchConfiguration('parameters_file').perform(context),
    )
    parameters_file = (
        '' if spec.parameters_file is None else str(spec.parameters_file)
    )
    side = spec.side
    return [
        LogInfo(msg=f'单手 RViz：model={spec.model_id}, side={side}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(retargeting_share / 'launch' / 'mediapipe_rviz.launch.py')
            ),
            launch_arguments={
                'model_id': spec.model_id,
                'model_side': side,
                'target_hand': side,
                'pose_topic': f'/{side}/mediapipe/hand_pose',
                'angles_topic': f'/{side}/mediapipe/human_joint_angles',
                'debug_image_topic': f'/{side}/mediapipe/debug_image',
                'target_joint_topic': (
                    f'/{side}/linkerhand/target_joint_states'
                ),
                'status_topic': f'/{side}/linkerhand/retargeting_status',
                'joint_states_topic': f'/{side}/joint_states',
                'description_package': spec.profile.description_package,
                'parameters_file': parameters_file,
                'device': LaunchConfiguration('device'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'camera_fps': LaunchConfiguration('camera_fps'),
                'processing_fps': LaunchConfiguration('processing_fps'),
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
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_id', default_value='l30'),
        DeclareLaunchArgument('side', default_value='left'),
        DeclareLaunchArgument('parameters_file', default_value=''),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('processing_fps', default_value='15.0'),
        DeclareLaunchArgument('mediapipe_show_preview', default_value='true'),
        DeclareLaunchArgument('mirror_preview', default_value='true'),
        DeclareLaunchArgument('use_one_euro_filter', default_value='true'),
        DeclareLaunchArgument('one_euro_min_cutoff', default_value='0.8'),
        DeclareLaunchArgument('one_euro_beta', default_value='0.3'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
