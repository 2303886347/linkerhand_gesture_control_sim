"""同时识别左右手，并在同一个 RViz 中显示任意已注册型号组合。"""

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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from linkerhand_retargeting.bringup import resolve_rviz_hand_spec


def _mediapipe_node(
    side,
    config_file,
    processing_fps,
    show_previews,
    mirror_preview,
    use_one_euro_filter,
    one_euro_min_cutoff,
    one_euro_beta,
):
    side_title = 'Left' if side == 'left' else 'Right'
    return Node(
        package='mediapipe_hand_pose',
        executable='hand_pose_node',
        namespace=side,
        name='mediapipe_hand_pose',
        output='screen',
        parameters=[
            config_file,
            {
                'input_topic': '/usb_camera/image_raw',
                'pose_topic': f'/{side}/mediapipe/hand_pose',
                'angles_topic': f'/{side}/mediapipe/human_joint_angles',
                'debug_image_topic': f'/{side}/mediapipe/debug_image',
                'target_hand': side,
                'max_processing_fps': ParameterValue(
                    processing_fps, value_type=float
                ),
                'show_preview': ParameterValue(
                    show_previews, value_type=bool
                ),
                'preview_window_name': f'MediaPipe {side_title} Hand',
                'mirror_preview': ParameterValue(
                    mirror_preview, value_type=bool
                ),
                'use_one_euro_filter': ParameterValue(
                    use_one_euro_filter, value_type=bool
                ),
                'one_euro_min_cutoff': ParameterValue(
                    one_euro_min_cutoff, value_type=float
                ),
                'one_euro_beta': ParameterValue(
                    one_euro_beta, value_type=float
                ),
            },
        ],
    )


def _retargeting_nodes(spec):
    side = spec.side
    target_topic = f'/{side}/linkerhand/target_joint_states'
    retargeting_parameters = []
    if spec.parameters_file is not None:
        retargeting_parameters.append(str(spec.parameters_file))
    retargeting_parameters.append({
        'model_id': spec.model_id,
        'model_side': side,
        'accepted_hand': side,
        'input_topic': f'/{side}/mediapipe/hand_pose',
        'output_topic': target_topic,
        'status_topic': f'/{side}/linkerhand/retargeting_status',
    })
    return [
        Node(
            package='linkerhand_retargeting',
            executable='retargeting_node',
            namespace=side,
            name='linkerhand_retargeting',
            output='screen',
            parameters=retargeting_parameters,
        ),
        Node(
            package='linkerhand_retargeting',
            executable='rviz_joint_state_adapter',
            namespace=side,
            name='linkerhand_rviz_joint_state_adapter',
            output='screen',
            parameters=[{
                'model_id': spec.model_id,
                'model_side': side,
                'input_topic': target_topic,
                'output_topic': f'/{side}/joint_states',
            }],
        ),
    ]


def _robot_state_publisher(side, robot_description):
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=side,
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'frame_prefix': f'{side}/',
            }
        ],
        remappings=[('joint_states', f'/{side}/joint_states')],
    )


def _mount_transform(side, root_link, y_position):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{side}_hand_mount',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', str(y_position),
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', f'{side}/{root_link}',
        ],
    )


def _launch_setup(context):
    camera_share = Path(get_package_share_directory('usb_camera_demo'))
    mediapipe_share = Path(get_package_share_directory('mediapipe_hand_pose'))
    retargeting_share = Path(
        get_package_share_directory('linkerhand_retargeting')
    )
    left_spec = resolve_rviz_hand_spec(
        LaunchConfiguration('left_model').perform(context),
        'left',
        retargeting_share,
        LaunchConfiguration('left_parameters_file').perform(context),
    )
    right_spec = resolve_rviz_hand_spec(
        LaunchConfiguration('right_model').perform(context),
        'right',
        retargeting_share,
        LaunchConfiguration('right_parameters_file').perform(context),
    )

    mediapipe_config = str(mediapipe_share / 'config' / 'hand_pose.yaml')

    camera = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(camera_share / 'launch' / 'usb_camera.launch.py')
                ),
                launch_arguments={
                    'device': LaunchConfiguration('device'),
                    'width': LaunchConfiguration('width'),
                    'height': LaunchConfiguration('height'),
                    'fps': LaunchConfiguration('camera_fps'),
                    'show_preview': 'false',
                }.items(),
            )
        ],
    )

    actions = [
        LogInfo(
            msg=(
                '多型号 RViz 组合：'
                f'left={left_spec.model_id}, right={right_spec.model_id}'
            )
        ),
    ]
    for spec in (left_spec, right_spec):
        if spec.uses_profile_defaults:
            get_logger('linkerhand_multi_model_rviz').warning(
                f'未找到 {spec.model_id}/{spec.side} 的默认标定 YAML；'
                '将回退到型号 profile 默认映射。可通过 '
                f'{spec.side}_parameters_file:=<yaml> 显式指定。'
            )

    actions.extend([
        camera,
        _mediapipe_node(
            'left',
            mediapipe_config,
            LaunchConfiguration('processing_fps'),
            LaunchConfiguration('show_previews'),
            LaunchConfiguration('mirror_preview'),
            LaunchConfiguration('use_one_euro_filter'),
            LaunchConfiguration('one_euro_min_cutoff'),
            LaunchConfiguration('one_euro_beta'),
        ),
        _mediapipe_node(
            'right',
            mediapipe_config,
            LaunchConfiguration('processing_fps'),
            LaunchConfiguration('show_previews'),
            LaunchConfiguration('mirror_preview'),
            LaunchConfiguration('use_one_euro_filter'),
            LaunchConfiguration('one_euro_min_cutoff'),
            LaunchConfiguration('one_euro_beta'),
        ),
        *_retargeting_nodes(left_spec),
        *_retargeting_nodes(right_spec),
        _robot_state_publisher('left', left_spec.robot_description),
        _robot_state_publisher('right', right_spec.robot_description),
        _mount_transform('left', left_spec.profile.root_link, 0.16),
        _mount_transform('right', right_spec.profile.root_link, -0.16),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_both_hands',
            arguments=[
                '-d',
                str(retargeting_share / 'rviz' / 'both_hands.rviz'),
            ],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'left_model',
            default_value='l30',
            description='左手机械手型号 profile，例如 l30 或 o6。',
        ),
        DeclareLaunchArgument(
            'right_model',
            default_value='l30',
            description='右手机械手型号 profile，例如 l30 或 o6。',
        ),
        DeclareLaunchArgument(
            'left_parameters_file',
            default_value='',
            description='左手个人标定 YAML；为空时使用型号默认配置。',
        ),
        DeclareLaunchArgument(
            'right_parameters_file',
            default_value='',
            description='右手个人标定 YAML；为空时使用型号默认配置。',
        ),
        DeclareLaunchArgument(
            'device', default_value='/dev/video0', description='摄像头设备路径。'
        ),
        DeclareLaunchArgument(
            'width', default_value='640', description='摄像头图像宽度。'
        ),
        DeclareLaunchArgument(
            'height', default_value='480', description='摄像头图像高度。'
        ),
        DeclareLaunchArgument(
            'camera_fps', default_value='30.0', description='摄像头请求帧率。'
        ),
        DeclareLaunchArgument(
            'processing_fps',
            default_value='10.0',
            description='每个 MediaPipe 实例的最大处理帧率。',
        ),
        DeclareLaunchArgument(
            'show_previews',
            default_value='true',
            description='是否分别显示左右手标注预览窗口。',
        ),
        DeclareLaunchArgument(
            'mirror_preview',
            default_value='true',
            description='是否镜像左右手标注预览。',
        ),
        DeclareLaunchArgument(
            'use_one_euro_filter',
            default_value='true',
            description='是否对左右手关键点和角度启用 One Euro 消抖。',
        ),
        DeclareLaunchArgument(
            'one_euro_min_cutoff',
            default_value='0.8',
            description='左右手 One Euro 静止平滑强度。',
        ),
        DeclareLaunchArgument(
            'one_euro_beta',
            default_value='0.3',
            description='左右手 One Euro 快速动作自适应系数。',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动双手 RViz。',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
