"""启动 MediaPipe 手部姿态识别节点。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = 'mediapipe_hand_pose'


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/usb_camera/image_raw',
            description='输入 ROS 2 图像话题。',
        ),
        DeclareLaunchArgument(
            'target_hand',
            default_value='any',
            description='目标手：any、left 或 right。',
        ),
        DeclareLaunchArgument(
            'input_mirrored',
            default_value='false',
            description='输入图像是否已经做过自拍镜像。',
        ),
        DeclareLaunchArgument(
            'max_processing_fps',
            default_value='15.0',
            description='MediaPipe 最大处理帧率，0 表示不限制。',
        ),
        DeclareLaunchArgument(
            'show_preview',
            default_value='true',
            description='是否显示带关键点和角度的 OpenCV 预览窗口。',
        ),
        DeclareLaunchArgument(
            'preview_window_name',
            default_value='MediaPipe Hand Pose',
            description='OpenCV 预览窗口名称。',
        ),
        DeclareLaunchArgument(
            'mirror_preview',
            default_value='true',
            description='是否左右镜像带标注的预览和调试图像。',
        ),
        DeclareLaunchArgument(
            'use_one_euro_filter',
            default_value='true',
            description='是否对显示关键点和人体关节角启用 One Euro 消抖。',
        ),
        DeclareLaunchArgument(
            'one_euro_min_cutoff',
            default_value='0.8',
            description='One Euro 静止平滑强度，越小越稳但延迟越大。',
        ),
        DeclareLaunchArgument(
            'one_euro_beta',
            default_value='0.3',
            description='One Euro 速度自适应系数，越大快速动作越灵敏。',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='是否发布带标注的调试图像。',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='hand_pose_node',
            name='mediapipe_hand_pose',
            output='screen',
            parameters=[
                str(package_share / 'config' / 'hand_pose.yaml'),
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                    'target_hand': LaunchConfiguration('target_hand'),
                    'input_mirrored': ParameterValue(
                        LaunchConfiguration('input_mirrored'), value_type=bool
                    ),
                    'max_processing_fps': ParameterValue(
                        LaunchConfiguration('max_processing_fps'), value_type=float
                    ),
                    'show_preview': ParameterValue(
                        LaunchConfiguration('show_preview'), value_type=bool
                    ),
                    'preview_window_name': LaunchConfiguration(
                        'preview_window_name'
                    ),
                    'mirror_preview': ParameterValue(
                        LaunchConfiguration('mirror_preview'), value_type=bool
                    ),
                    'use_one_euro_filter': ParameterValue(
                        LaunchConfiguration('use_one_euro_filter'), value_type=bool
                    ),
                    'one_euro_min_cutoff': ParameterValue(
                        LaunchConfiguration('one_euro_min_cutoff'), value_type=float
                    ),
                    'one_euro_beta': ParameterValue(
                        LaunchConfiguration('one_euro_beta'), value_type=float
                    ),
                    'publish_debug_image': ParameterValue(
                        LaunchConfiguration('publish_debug_image'), value_type=bool
                    ),
                },
            ],
        ),
    ])
