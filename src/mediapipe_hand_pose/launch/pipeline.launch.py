"""同时启动 USB 摄像头和 MediaPipe 手部姿态识别。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_share = Path(get_package_share_directory('usb_camera_demo'))
    mediapipe_share = Path(get_package_share_directory('mediapipe_hand_pose'))

    # 两个子 launch 都有 show_preview 参数，必须使用独立作用域防止互相覆盖。
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
                    'show_preview': LaunchConfiguration('camera_show_preview'),
                }.items(),
            )
        ],
    )
    hand_pose = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(mediapipe_share / 'launch' / 'mediapipe.launch.py')
                ),
                launch_arguments={
                    'target_hand': LaunchConfiguration('target_hand'),
                    'max_processing_fps': LaunchConfiguration('processing_fps'),
                    'show_preview': LaunchConfiguration('mediapipe_show_preview'),
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
            default_value='any',
            description='目标手：any、left 或 right。',
        ),
        DeclareLaunchArgument(
            'show_preview',
            default_value='true',
            description='兼容参数：MediaPipe 标注预览窗口的默认开关。',
        ),
        DeclareLaunchArgument(
            'camera_show_preview',
            default_value='false',
            description='是否显示未处理的摄像头预览窗口。',
        ),
        DeclareLaunchArgument(
            'mediapipe_show_preview',
            default_value=LaunchConfiguration('show_preview'),
            description='是否显示 MediaPipe 标注预览窗口。',
        ),
        DeclareLaunchArgument(
            'mirror_preview',
            default_value='true',
            description='是否左右镜像 MediaPipe 标注预览和调试图像。',
        ),
        DeclareLaunchArgument(
            'use_one_euro_filter',
            default_value='true',
            description='是否启用 MediaPipe 关键点和角度消抖。',
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
        camera,
        hand_pose,
    ])
