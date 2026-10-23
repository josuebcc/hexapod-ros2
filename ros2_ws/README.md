# Espacio de trabajo ROS 2

## Paquetes

```text
src/
├── hexapod_bridge/   Adaptador entre topics ROS 2 y la UART del ESP32
├── hexapod_web/      Panel web de control y supervisión
├── hexapod_bringup/  Launch y parámetros comunes del sistema
├── obstacle_avoider/ Evitador reactivo de obstáculos a partir del LiDAR
└── sllidar_ros2/     Driver externo del LiDAR Slamtec
```

Esta separación mantiene el código propio limitado a las interfaces del robot.
El driver del LiDAR se integra como un paquete independiente, y el filtrado de
su barrido usa el paquete `laser_filters` de ROS 2, que instala `rosdep`.

`obstacle_avoider` calcula propuestas continuamente. La web arranca en manual y
es la unica que envia velocidades al puente: propias en manual, o las del
evitador al activar la autonomia desde el panel.
Su [README](src/obstacle_avoider/README.md) detalla el algoritmo y la puesta en marcha.

## Compilación

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Arranque completo:

```bash
ros2 launch hexapod_bringup hexapod.launch.py
```

## Comunicación con RViz en otro ordenador

Este workspace usa el dominio ROS 2 `25`. En ambos ordenadores —el del robot y
el que ejecuta RViz— hay que cargar:

```bash
export ROS_DOMAIN_ID=25
source /opt/ros/jazzy/setup.bash
```

La configuración queda guardada en el `~/.bashrc` de este ordenador. En el
ordenador del robot debe usarse también el dominio `25`.

Los directorios `build/`, `install/` y `log/` son locales y no se versionan.
