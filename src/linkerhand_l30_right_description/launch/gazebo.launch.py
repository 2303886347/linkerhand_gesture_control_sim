"""在 Gazebo Sim 中生成 Linker Hand L30 右手模型。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


PACKAGE_NAME = 'linkerhand_l30_right_description'


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                world,
                ' -r',
                # 无界面模式只启动仿真服务器，适合远程终端和自动测试。
                PythonExpression([
                    "' -s' if '",
                    LaunchConfiguration('headless'),
                    "'.lower() == 'true' else ''",
                ]),
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_linkerhand_l30_right',
        arguments=[
            '-file', model,
            '-name', 'linkerhand_l30_right',
            '-allow_renaming', 'true',
            '-z', LaunchConfiguration('z'),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=str(package_share / 'urdf' / 'linkerhand_l30_right.urdf'),
            description='URDF 文件的绝对路径。',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Gazebo Sim 世界名称或 SDF 文件的绝对路径。',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.01',
            description='模型生成时的初始高度，单位为米。',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='是否仅运行 Gazebo 服务器而不启动图形客户端。',
        ),
        gazebo,
        TimerAction(period=3.0, actions=[spawn_model]),
    ])
