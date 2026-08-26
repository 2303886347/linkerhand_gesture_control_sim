from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = 'linkerhand_model_profiles'


setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md', 'pytest.ini']),
        (
            'share/' + PACKAGE_NAME + '/config/l30',
            glob('config/l30/*.yaml'),
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='2303886347',
    maintainer_email='2303886347@qq.com',
    description='统一加载并校验 Linker Hand 型号配置。',
    license='Apache-2.0',
)
