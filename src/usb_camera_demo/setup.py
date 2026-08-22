from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = 'usb_camera_demo'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md']),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='2303886347',
    maintainer_email='2303886347@qq.com',
    description='最小 ROS 2 USB 摄像头采集与图像发布示例。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'usb_camera_node = usb_camera_demo.usb_camera_node:main',
        ],
    },
)
