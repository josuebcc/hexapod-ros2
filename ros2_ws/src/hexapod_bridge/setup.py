import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hexapod_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='jbc',
    maintainer_email='josuebh2003@outlook.com',
    description='Puente serie entre ROS 2 y el controlador ESP32 del hexapodo',
    license='NOASSERTION',
    entry_points={
        'console_scripts': [
            'esp32_bridge = hexapod_bridge.esp32_bridge:main',
        ],
    },
)
