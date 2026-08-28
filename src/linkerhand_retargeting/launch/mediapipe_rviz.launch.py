"""启动摄像头、MediaPipe、角度转换和 Linker Hand RViz 同步显示。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mediapipe_share = Path(get_package_share_directory('mediapipe_hand_pose'))
    retargeting_share = Path(get_package_share_directory('linkerhand_retargeting'))

    # 子 launch 使用独立作用域，避免同名参数在不同包之间互相覆盖。
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
                    'target_hand': LaunchConfiguration('target_hand'),
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
                    str(retargeting_share / 'launch' / 'retargeting.launch.py')
                ),
                launch_arguments={
                    'parameters_file': LaunchConfiguration('parameters_file'),
                    'model_id': LaunchConfiguration('model_id'),
                    'model_side': LaunchConfiguration('model_side'),
                    'input_topic': LaunchConfiguration('pose_topic'),
                    'output_topic': LaunchConfiguration('target_joint_topic'),
                    'status_topic': LaunchConfiguration('status_topic'),
                    'joint_states_topic': LaunchConfiguration(
                        'joint_states_topic'
                    ),
                    'use_rviz_adapter': 'true',
                }.items(),
            )
        ],
    )

    visualization = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare(
                            LaunchConfiguration('description_package')
                        ),
                        'launch',
                        'display.launch.py',
                    ])
                ),
                launch_arguments={
                    'use_gui': 'false',
                    'use_rviz': LaunchConfiguration('use_rviz'),
                    'joint_states_topic': LaunchConfiguration(
                        'joint_states_topic'
                    ),
                }.items(),
            )
        ],
    )

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
            default_value='15.0',
            description='MediaPipe 最大处理帧率。',
        ),
        DeclareLaunchArgument(
            'target_hand',
            default_value='left',
            description='跟踪目标手：left、right 或 any。',
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/left/mediapipe/hand_pose',
            description='完整手部姿态输出话题。',
        ),
        DeclareLaunchArgument(
            'angles_topic',
            default_value='/left/mediapipe/human_joint_angles',
            description='人体关节角输出话题。',
        ),
        DeclareLaunchArgument(
            'debug_image_topic',
            default_value='/left/mediapipe/debug_image',
            description='带标注调试图像输出话题。',
        ),
        DeclareLaunchArgument(
            'target_joint_topic',
            default_value='/left/linkerhand/target_joint_states',
            description='机械手独立关节目标话题。',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/left/linkerhand/retargeting_status',
            description='重定向状态话题。',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/left/joint_states',
            description='RViz 完整关节状态话题。',
        ),
        DeclareLaunchArgument(
            'description_package',
            default_value='linkerhand_l30_left_description',
            description='RViz 使用的左手或右手模型描述包。',
        ),
        DeclareLaunchArgument(
            'mediapipe_show_preview',
            default_value='true',
            description='是否显示 MediaPipe 标注预览窗口。',
        ),
        DeclareLaunchArgument(
            'mirror_preview',
            default_value='true',
            description='是否镜像 MediaPipe 标注预览和调试图像。',
        ),
        DeclareLaunchArgument(
            'use_one_euro_filter',
            default_value='true',
            description='是否启用关键点和人体关节角消抖。',
        ),
        DeclareLaunchArgument(
            'one_euro_min_cutoff',
            default_value='0.8',
            description='One Euro 静止平滑强度。',
        ),
        DeclareLaunchArgument(
            'one_euro_beta',
            default_value='0.3',
            description='One Euro 快速动作自适应系数。',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', description='是否启动 RViz 2。'
        ),
        DeclareLaunchArgument(
            'parameters_file',
            default_value=str(
                retargeting_share / 'config' / 'retargeting_left.yaml'
            ),
            description='角度映射和滤波参数文件。',
        ),
        DeclareLaunchArgument(
            'model_id',
            default_value='',
            description='可选的型号覆盖；为空时使用参数文件值。',
        ),
        DeclareLaunchArgument(
            'model_side',
            default_value='',
            description='可选的手侧覆盖；为空时使用参数文件值。',
        ),
        perception,
        retargeting,
        visualization,
    ])
