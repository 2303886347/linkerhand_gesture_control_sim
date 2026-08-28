"""在 RViz 2 中独立检查 Linker Hand O6 右手模型。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = 'linkerhand_o6_right_description'


def _launch_setup(context):
    model_path = Path(LaunchConfiguration('model').perform(context))
    robot_description = model_path.read_text(encoding='utf-8')
    return [
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace='right',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_gui')),
            remappings=[
                ('joint_states', LaunchConfiguration('joint_states_topic'))
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='right',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': robot_description,
                    'frame_prefix': 'right/',
                }
            ],
            remappings=[
                ('joint_states', LaunchConfiguration('joint_states_topic'))
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_o6_right_hand',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=str(package_share / 'urdf' / 'linkerhand_o6_right.urdf'),
            description='O6 右手 URDF 文件的绝对路径。',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(package_share / 'rviz' / 'display.rviz'),
            description='RViz 2 配置文件的绝对路径。',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/right/joint_states',
            description='O6 右手完整关节状态话题。',
        ),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
