"""启动 Linker Hand 个人标定 Qt 上位机。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linkerhand_calibration',
            executable='calibration_gui',
            name='linkerhand_calibration_gui',
            output='screen',
            emulate_tty=True,
        ),
    ])
