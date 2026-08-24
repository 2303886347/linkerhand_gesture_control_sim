"""启动手部角度转换节点，并可选发布 RViz 完整关节状态。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('linkerhand_retargeting'))
    default_parameters = str(
        package_share / 'config' / 'retargeting_left.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters_file',
            default_value=default_parameters,
            description='角度映射参数文件。',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/mediapipe/hand_pose',
            description='MediaPipe 手部姿态输入话题。',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/linkerhand/target_joint_states',
            description='机械手独立关节目标话题。',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/linkerhand/retargeting_status',
            description='重定向状态话题。',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='RViz 完整关节状态输出话题。',
        ),
        DeclareLaunchArgument(
            'use_rviz_adapter',
            default_value='false',
            description='是否把独立关节目标扩展后发布到 /joint_states。',
        ),
        Node(
            package='linkerhand_retargeting',
            executable='retargeting_node',
            name='linkerhand_retargeting',
            output='screen',
            parameters=[
                LaunchConfiguration('parameters_file'),
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                    'output_topic': LaunchConfiguration('output_topic'),
                    'status_topic': LaunchConfiguration('status_topic'),
                },
            ],
        ),
        Node(
            package='linkerhand_retargeting',
            executable='rviz_joint_state_adapter',
            name='linkerhand_rviz_joint_state_adapter',
            output='screen',
            parameters=[
                {
                    'input_topic': LaunchConfiguration('output_topic'),
                    'output_topic': LaunchConfiguration('joint_states_topic'),
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_rviz_adapter')),
        ),
    ])
