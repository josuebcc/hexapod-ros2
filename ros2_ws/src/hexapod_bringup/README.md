# hexapod_bringup

Configuración y lanzamiento conjunto del puente ESP32, el panel web, el driver
del LiDAR, el filtrado del barrido y el evitador reactivo. Los nodos con
configuración YAML la conservan en su propio paquete; los filtros del LiDAR, que
usan un paquete externo, la tienen aquí:

- `hexapod_bridge/config/esp32_bridge.yaml`
- `hexapod_web/config/web_panel.yaml`
- `hexapod_bringup/config/scan_filters.yaml`

```bash
ros2 launch hexapod_bringup hexapod.launch.py
```

El argumento `start_web` permite omitir el panel y `start_lidar` permite arrancar
sin el sensor.

`hexapod.launch.py` arranca también `scan_to_scan_filter_chain` (paquete
`laser_filters`), que limpia `/lidar/scan` y publica `/lidar/scan_filtered`, y
`reactive_node`, que consume `/lidar/scan_filtered` y publica propuestas
`geometry_msgs/msg/Twist` en `/obstacle_avoider/cmd_vel`. El launch remapea los
topics del LiDAR bajo `/lidar` para que `rqt_graph` los agrupe; los nodos siguen
en la raíz. El argumento `scan_filters_file` permite usar otra configuración de
filtros. La web
recibe las propuestas y solo las reenvía a `/hexapod/cmd_vel` en modo autónomo;
en manual envía las velocidades de sus controles. La web arranca en manual y
Stop cancela la autonomía. No hay nodo selector.

Para ejecutar solo el evitador, con entrada `/lidar/scan_filtered` y salida `/obstacle_avoider/cmd_vel`:

```bash
ros2 run obstacle_avoider reactive_node --ros-args -r /scan_filtered:=/lidar/scan_filtered
```
