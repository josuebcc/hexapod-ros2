from setuptools import find_packages, setup

package_name = 'obstacle_avoider'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='jbc',
    maintainer_email='josuebh2003@outlook.com',
    description='Evitador reactivo de obstaculos basado en sectores LiDAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'reactive_node = obstacle_avoider.reactive_node:main',
        ],
    },
)
