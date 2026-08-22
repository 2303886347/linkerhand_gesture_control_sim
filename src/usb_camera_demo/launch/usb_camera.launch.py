"""启动最小 USB 摄像头图像发布节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='/dev/video0', description='摄像头设备路径。'
        ),
        DeclareLaunchArgument(
            'width', default_value='640', description='请求的图像宽度。'
        ),
        DeclareLaunchArgument(
            'height', default_value='480', description='请求的图像高度。'
        ),
        DeclareLaunchArgument(
            'fps', default_value='30.0', description='请求的采集帧率。'
        ),
        DeclareLaunchArgument(
            'show_preview', default_value='true', description='是否显示 OpenCV 预览窗口。'
        ),
        Node(
            package='usb_camera_demo',
            executable='usb_camera_node',
            name='usb_camera',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
                'fps': ParameterValue(LaunchConfiguration('fps'), value_type=float),
                'show_preview': ParameterValue(
                    LaunchConfiguration('show_preview'), value_type=bool
                ),
            }],
        ),
    ])
