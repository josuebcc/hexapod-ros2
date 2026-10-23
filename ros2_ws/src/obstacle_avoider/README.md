# obstacle_avoider: evitador reactivo

Nodo ROS 2 sencillo para evitar obstáculos con el LiDAR. No construye mapas ni
planifica trayectorias: en cada barrido toma una única decisión.

La estrategia está inspirada conceptualmente en los controladores reactivos de
tres sectores, como `NOAA-dev/ros2-obstacle-avoider`, pero la implementación se
ha escrito para este proyecto y para la geometría de este hexápodo:

https://github.com/NOAA-dev/ros2-obstacle-avoider

## Geometría

El robot se modela como un círculo de 0.35 m de radio centrado en el LiDAR. Los
pies están a 0.240-0.248 m del centro en la postura neutral y no pasan de
0.294 m con el paso máximo, así que el círculo los contiene. Se supone que el
sensor está aproximadamente centrado en el cuerpo.

## Comportamiento

En cada barrido se calculan cuatro distancias, todas ellas la medida más
cercana de su zona:

- **frente**: pasillo de 0.70 m de ancho (el diámetro del robot) que el robot
  barrería al avanzar; es la menor `x` de los puntos con `|y| <= 0.35 m`;
- **izquierda**: sector de +35 a +120 grados;
- **derecha**: sector de -120 a -35 grados;
- **alrededor**: todas las direcciones.

Un sector sin medidas válidas se considera ocupado. Con esas distancias:

1. **Avanzar.** Si el pasillo tiene al menos 0.50 m libres, avanza a 0.08 m/s y
   se separa de las paredes laterales a menos de 0.80 m con un desplazamiento
   lateral proporcional a la diferencia entre ambos lados (máximo 0.015 m/s),
   sin cambiar el rumbo. Con trípode y periodo de 2 s, el puente admite
   0.12 m/s en X y 0.05 m/s en Y; avance y corrección suman como máximo el
   97 % de esos límites, así que el puente no reduce la velocidad de avance.
2. **Girar.** Si el pasillo está ocupado y el círculo del robot está libre,
   gira sobre sí mismo a 0.25 rad/s hacia el lado con más espacio hasta que el
   pasillo vuelve a estar libre. El lado se elige al empezar el giro y se
   mantiene hasta volver a avanzar, y para volver a avanzar exige 0.60 m en vez
   de 0.50 m. Esa histéresis evita que alterne entre avanzar y girar, o entre
   ambos sentidos, por pequeñas variaciones entre barridos.
3. **Parar.** Si hay alguna medida dentro del círculo del robot, publica
   velocidad cero: girar podría llevar una pata contra el obstáculo.

| Parámetro | Valor |
| --- | --- |
| Radio del robot (`robot_radius`) | 0.35 m |
| Pasillo libre para seguir avanzando (`front_clearance`) | 0.50 m |
| Pasillo libre para volver a avanzar tras girar (`resume_clearance`) | 0.60 m |
| Alcance de la corrección lateral (`side_influence`) | 0.80 m |
| Velocidad de avance (`forward_speed`) | 0.08 m/s |
| Velocidad lateral máxima al avanzar (`lateral_speed`) | 0.015 m/s |
| Velocidad de giro (`turn_speed`) | 0.25 rad/s |

## Filtrado previo del LiDAR

El nodo no lee `/lidar/scan` directamente, sino `/lidar/scan_filtered`, que publica
`scan_to_scan_filter_chain` (paquete `laser_filters`) con la configuración de
`hexapod_bringup/config/scan_filters.yaml`:

1. `InterpolationFilter` rellena cada tramo sin eco (`inf`) con la media de las
   medidas válidas que lo delimitan. Con el RPLIDAR A1, un tramo sin eco en
   interiores suele ser una superficie oscura o transparente o algo a menos de
   0.15 m, no espacio libre.
2. `LaserScanSpeckleFilter` descarta (`NaN`) las medidas a menos de 1 m que no
   forman parte de un grupo de tres medidas consecutivas que difieren menos de
   0.1 m.

El orden importa: si se quitaran primero las medidas sueltas, los ecos aislados
de un objeto oscuro desaparecerían antes de servir de referencia a la
interpolación. Los `NaN` que quedan los ignora el evitador.

## Topics e integración

- Entrada: `/lidar/scan_filtered`, `sensor_msgs/msg/LaserScan`. El nodo se
  suscribe a `/scan_filtered` y `hexapod.launch.py` lo remapea.
- Salida: `/obstacle_avoider/cmd_vel`, `geometry_msgs/msg/Twist`.

El nodo calcula siempre, pero no publica directamente al robot. El panel web
reenvía `/obstacle_avoider/cmd_vel` a `/hexapod/cmd_vel` solamente cuando está
en modo autónomo. La web arranca en modo manual.

## Compilar y ejecutar

Desde la raíz de `ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # instala laser_filters
colcon build --packages-up-to hexapod_bringup
source install/setup.bash
ros2 launch hexapod_bringup hexapod.launch.py
```

Para ejecutar solamente el evitador (necesita que el filtro esté publicando
`/lidar/scan_filtered`):

```bash
ros2 run obstacle_avoider reactive_node --ros-args -r /scan_filtered:=/lidar/scan_filtered
```

Para observar sus propuestas:

```bash
ros2 topic echo /obstacle_avoider/cmd_vel
```

Tests:

```bash
colcon test --packages-select obstacle_avoider && colcon test-result --verbose
```
