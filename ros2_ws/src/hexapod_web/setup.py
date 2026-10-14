import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hexapod_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    package_data={
        package_name: [
            'templates/*.html',
            'static/css/*.css',
            'static/js/*.js',
            'static/js/panel/*.js',
            'static/js/servos/*.js',
            'static/js/shared/*.js',
        ],
    },
    install_requires=[
        'setuptools',
        'flask',
    ],
    zip_safe=True,
    maintainer='jbc',
    maintainer_email='josuebh2003@outlook.com',
    description='Panel web ROS 2 para supervisar y controlar el hexápodo',
    license='NOASSERTION',
    entry_points={
        'console_scripts': [
            'web_panel = hexapod_web.web_server:main',
        ],
    },
)
