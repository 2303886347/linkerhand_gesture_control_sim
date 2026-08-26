"""同时识别左右手，并在同一个 RViz 中显示两个 Linker Hand 模型。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def _retargeting_nodes(side, parameters_file):
    target_topic = f'/{side}/linkerhand/target_joint_states'
    return [
        Node(
            package='linkerhand_retargeting',
            executable='retargeting_node',
            namespace=side,
            name='linkerhand_retargeting',
            output='screen',
            parameters=[
                parameters_file,
                {
                    'input_topic': f'/{side}/mediapipe/hand_pose',
                    'output_topic': target_topic,
                    'status_topic': f'/{side}/linkerhand/retargeting_status',
                },
            ],
        ),
        Node(
            package='linkerhand_retargeting',
            executable='rviz_joint_state_adapter',
            namespace=side,
            name='linkerhand_rviz_joint_state_adapter',
            output='screen',
            parameters=[
                parameters_file,
                {
                    'input_topic': target_topic,
                    'output_topic': f'/{side}/joint_states',
                }
            ],
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


def _mount_transform(side, y_position):
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
            '--child-frame-id', f'{side}/base_footprint',
        ],
    )


def generate_launch_description():
    camera_share = Path(get_package_share_directory('usb_camera_demo'))
    mediapipe_share = Path(get_package_share_directory('mediapipe_hand_pose'))
    retargeting_share = Path(get_package_share_directory('linkerhand_retargeting'))
    left_share = Path(
        get_package_share_directory('linkerhand_l30_left_description')
    )
    right_share = Path(
        get_package_share_directory('linkerhand_l30_right_description')
    )

    mediapipe_config = str(mediapipe_share / 'config' / 'hand_pose.yaml')
    left_parameters = str(
        retargeting_share / 'config' / 'retargeting_left.yaml'
    )
    right_parameters = str(
        retargeting_share / 'config' / 'retargeting_right.yaml'
    )
    left_description = (
        left_share / 'urdf' / 'linkerhand_l30_left.urdf'
    ).read_text(encoding='utf-8')
    right_description = (
        right_share / 'urdf' / 'linkerhand_l30_right.urdf'
    ).read_text(encoding='utf-8')

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

    left_retargeting = _retargeting_nodes('left', left_parameters)
    right_retargeting = _retargeting_nodes('right', right_parameters)

    return LaunchDescription([
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
        *left_retargeting,
        *right_retargeting,
        _robot_state_publisher('left', left_description),
        _robot_state_publisher('right', right_description),
        _mount_transform('left', 0.16),
        _mount_transform('right', -0.16),
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
