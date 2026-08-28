from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = 'linkerhand_gazebo_control'


setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md', 'pytest.ini']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE_NAME + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='2303886347',
    maintainer_email='2303886347@qq.com',
    description='将 Linker Hand 手势关节目标同步到 Gazebo Sim 并反馈状态。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_adapter = '
            'linkerhand_gazebo_control.trajectory_adapter:main',
            'joint_state_throttle = '
            'linkerhand_gazebo_control.joint_state_throttle:main',
        ],
    },
)
