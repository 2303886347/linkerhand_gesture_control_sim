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
            'use_rviz_adapter',
            default_value='false',
            description='是否把独立关节目标扩展后发布到 /joint_states。',
        ),
        Node(
            package='linkerhand_retargeting',
            executable='retargeting_node',
            name='linkerhand_retargeting',
            output='screen',
            parameters=[LaunchConfiguration('parameters_file')],
        ),
        Node(
            package='linkerhand_retargeting',
            executable='rviz_joint_state_adapter',
            name='linkerhand_rviz_joint_state_adapter',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz_adapter')),
        ),
    ])
