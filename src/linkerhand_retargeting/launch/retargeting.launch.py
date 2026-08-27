"""启动手部角度转换节点，并可选发布 RViz 完整关节状态。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    parameters_file = LaunchConfiguration('parameters_file').perform(context)
    model_id = LaunchConfiguration('model_id').perform(context).strip()
    model_side = LaunchConfiguration('model_side').perform(context).strip()

    shared_overrides = {
        'input_topic': LaunchConfiguration('input_topic'),
        'output_topic': LaunchConfiguration('output_topic'),
        'status_topic': LaunchConfiguration('status_topic'),
    }
    adapter_overrides = {
        'input_topic': LaunchConfiguration('output_topic'),
        'output_topic': LaunchConfiguration('joint_states_topic'),
    }
    if model_id:
        shared_overrides['model_id'] = model_id
        adapter_overrides['model_id'] = model_id
    if model_side:
        shared_overrides['model_side'] = model_side
        adapter_overrides['model_side'] = model_side

    node_parameters = []
    adapter_parameters = []
    if parameters_file.strip():
        node_parameters.append(parameters_file)
        adapter_parameters.append(parameters_file)
    node_parameters.append(shared_overrides)
    adapter_parameters.append(adapter_overrides)

    return [
        Node(
            package='linkerhand_retargeting',
            executable='retargeting_node',
            name='linkerhand_retargeting',
            output='screen',
            parameters=node_parameters,
        ),
        Node(
            package='linkerhand_retargeting',
            executable='rviz_joint_state_adapter',
            name='linkerhand_rviz_joint_state_adapter',
            output='screen',
            parameters=adapter_parameters,
            condition=IfCondition(LaunchConfiguration('use_rviz_adapter')),
        ),
    ]


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
        OpaqueFunction(function=_launch_setup),
    ])
