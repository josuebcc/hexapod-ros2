import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def launch_arg(name, default_value, description):
    return DeclareLaunchArgument(
        name,
        default_value=default_value,
        description=description,
    )


def generate_launch_description():
    bringup_share_dir = get_package_share_directory('hexapod_bringup')
    bridge_share_dir = get_package_share_directory('hexapod_bridge')
    web_share_dir = get_package_share_directory('hexapod_web')
    sllidar_share_dir = get_package_share_directory('sllidar_ros2')

    default_bridge_params = os.path.join(
        bridge_share_dir,
        'config',
        'esp32_bridge.yaml',
    )
    default_web_params = os.path.join(web_share_dir, 'config', 'web_panel.yaml')
    default_scan_filters = os.path.join(
        bringup_share_dir,
        'config',
        'scan_filters.yaml',
    )
    default_lidar_launch = os.path.join(
        sllidar_share_dir,
        'launch',
        'sllidar_a1_launch.py',
    )

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    start_web = LaunchConfiguration('start_web')
    start_lidar = LaunchConfiguration('start_lidar')
    bridge_params_file = LaunchConfiguration('bridge_params_file')
    web_params_file = LaunchConfiguration('web_params_file')
    scan_filters_file = LaunchConfiguration('scan_filters_file')

    actions = [
        launch_arg(
            'lidar_port',
            '/dev/ttyUSB0',
            'Puerto serie conectado al LiDAR',
        ),
        launch_arg(
            'lidar_baud',
            '115200',
            'Baudrate del LiDAR',
        ),
        launch_arg(
            'start_web',
            'true',
            'Lanza tambien la interfaz web',
        ),
        launch_arg(
            'start_lidar',
            'true',
            'Lanza tambien el driver del LiDAR',
        ),
        launch_arg(
            'bridge_params_file',
            default_bridge_params,
            'Archivo YAML del puente ESP32',
        ),
        launch_arg(
            'web_params_file',
            default_web_params,
            'Archivo YAML del panel web',
        ),
        launch_arg(
            'scan_filters_file',
            default_scan_filters,
            'Archivo YAML de los filtros del LiDAR',
        ),
        Node(
            package='hexapod_bridge',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[bridge_params_file],
        ),
        Node(
            package='hexapod_web',
            executable='web_panel',
            name='web_panel',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(start_web),
            parameters=[web_params_file],
        ),
        # Los topics del LiDAR se agrupan bajo /lidar (rqt_graph los dibuja en
        # su propio bloque); los nodos se quedan en la raíz, como el resto.
        # Limpia /lidar/scan y publica /lidar/scan_filtered, que es lo que usa el evitador.
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            output='screen',
            parameters=[scan_filters_file],
            remappings=[
                ('scan', '/lidar/scan'),
                ('scan_filtered', '/lidar/scan_filtered'),
            ],
        ),
        Node(
            package='obstacle_avoider',
            executable='reactive_node',
            name='reactive_node',
            output='screen',
            remappings=[('/scan_filtered', '/lidar/scan_filtered')],
        ),
        GroupAction([
            SetRemap(src='scan', dst='/lidar/scan'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(default_lidar_launch),
                condition=IfCondition(start_lidar),
                launch_arguments={
                    'serial_port': lidar_port,
                    'serial_baudrate': lidar_baud,
                }.items(),
            ),
        ]),
    ]

    return LaunchDescription(actions)
