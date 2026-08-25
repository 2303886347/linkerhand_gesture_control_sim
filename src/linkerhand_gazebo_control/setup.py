from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = 'linkerhand_gazebo_control'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md', 'pytest.ini']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='2303886347',
    maintainer_email='2303886347@qq.com',
    description='将 Linker Hand 在线关节目标稳定同步到 Gazebo Sim。',
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
