from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = 'linkerhand_retargeting'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE_NAME + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='2303886347',
    maintainer_email='2303886347@qq.com',
    description='将 MediaPipe 人手角度映射为 Linker Hand 关节目标。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'retargeting_node = linkerhand_retargeting.retargeting_node:main',
            'rviz_joint_state_adapter = '
            'linkerhand_retargeting.rviz_joint_state_adapter:main',
        ],
    },
)
