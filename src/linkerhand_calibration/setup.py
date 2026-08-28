from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = 'linkerhand_calibration'


setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md']),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='2303886347',
    maintainer_email='2303886347@qq.com',
    description='通过 MediaPipe 姿态采样生成 Linker Hand 个人标定 YAML。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'calibration_gui = '
            'linkerhand_calibration.calibration_gui:main',
        ],
    },
)
