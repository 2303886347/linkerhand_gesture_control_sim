"""启动单只 Linker Hand 的 Gazebo 运动学位置反馈同步。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from linkerhand_gazebo_control.urdf_builder import build_controlled_urdf
from linkerhand_model_profiles import load_model_profile


def _as_bool(value):
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _launch_setup(context):
    side = LaunchConfiguration('side').perform(context).strip().lower()
    if side not in {'left', 'right'}:
        raise ValueError('side 只能是 left 或 right')
    model_id = LaunchConfiguration('model_id').perform(context).strip().lower()
    profile = load_model_profile(model_id, side)

    description_package = profile.description_package
    get_package_share_directory(description_package)
    model_name = profile.robot_name

    model_value = LaunchConfiguration('model').perform(context).strip()
    model_path = (
        Path(model_value)
        if model_value
        else profile.urdf_path
    )
    inertial_scale_value = LaunchConfiguration(
        'inertial_scale'
    ).perform(context).strip()
    inertial_scale = (
        float(inertial_scale_value) if inertial_scale_value else None
    )
    robot_description = build_controlled_urdf(
        model_path,
        side,
        inertial_scale=inertial_scale,
        profile=profile,
    )

    world = LaunchConfiguration('world').perform(context)
    headless = _as_bool(LaunchConfiguration('headless').perform(context))
    gazebo_arguments = f'{world} -r -v 2'
    if headless:
        gazebo_arguments += ' -s'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory('ros_gz_sim'))
                / 'launch'
                / 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': gazebo_arguments,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=side,
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    robot_description, value_type=str
                ),
                'frame_prefix': f'{side}/',
                'use_sim_time': True,
            }
        ],
        remappings=[('joint_states', f'/{side}/joint_states')],
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{model_name}',
        output='screen',
        arguments=[
            '-string',
            robot_description,
            '-name',
            model_name,
            '-allow_renaming',
            'false',
            '-x',
            LaunchConfiguration('x'),
            '-y',
            LaunchConfiguration('y'),
            '-z',
            LaunchConfiguration('z'),
        ],
    )

    trajectory_adapter = Node(
        package='linkerhand_gazebo_control',
        executable='trajectory_adapter',
        namespace=side,
        name='linkerhand_gazebo_trajectory_adapter',
        output='screen',
        parameters=[
            {
                'input_topic': f'/{side}/linkerhand/target_joint_states',
                'command_topic': f'/{side}/gazebo_joint_trajectory',
                'trajectory_duration': ParameterValue(
                    LaunchConfiguration('trajectory_duration'),
                    value_type=float,
                ),
                'model_id': model_id,
                'model_side': side,
            }
        ],
    )

    joint_state_throttle = Node(
        package='linkerhand_gazebo_control',
        executable='joint_state_throttle',
        namespace=side,
        name='linkerhand_gazebo_joint_state_throttle',
        output='screen',
        parameters=[
            {
                'input_topic': f'/{side}/gazebo_joint_states_raw',
                'output_topic': f'/{side}/joint_states',
                'publish_rate': 60.0,
            }
        ],
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{side}_gazebo_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            (
                f'/{side}/gazebo_joint_states_raw'
                '@sensor_msgs/msg/JointState'
                f'[gz.msgs.Model'
            ),
            (
                f'/{side}/gazebo_joint_trajectory'
                '@trajectory_msgs/msg/JointTrajectory'
                ']gz.msgs.JointTrajectory'
            ),
        ],
    )

    return [
        gazebo,
        robot_state_publisher,
        spawn_model,
        trajectory_adapter,
        joint_state_throttle,
        gazebo_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'side', default_value='left', description='控制 left 或 right 手。'
        ),
        DeclareLaunchArgument(
            'model_id', default_value='l30', description='机械手型号 profile。'
        ),
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='可选的原始 URDF 绝对路径。',
        ),
        DeclareLaunchArgument(
            'world', default_value='empty.sdf', description='Gazebo 世界。'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='是否只启动 Gazebo 服务端。',
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument(
            'trajectory_duration',
            default_value='0.15',
            description=(
                '轨迹消息的到达时间标记，单位为秒；当前插件的实际响应'
                '由 position_gain 和 max_velocity 决定。'
            ),
        ),
        DeclareLaunchArgument(
            'inertial_scale',
            default_value='',
            description=(
                '质量和惯量缩放；留空时左手为 1/7.6，右手为 1.0。'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
