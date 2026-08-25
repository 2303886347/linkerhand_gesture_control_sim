from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = 'mediapipe_hand_pose'


setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='2303886347',
    maintainer_email='2303886347@qq.com',
    description='基于 MediaPipe 输出左右手关键点、关节角和滤波调试图像。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hand_pose_node = mediapipe_hand_pose.hand_pose_node:main',
        ],
    },
)
