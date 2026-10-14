# hexapod_bridge

Puente de comunicaciones entre ROS 2 y el firmware del ESP32. Convierte
`hexapod/cmd_vel` y los comandos publicados en `hexapod/robot_command` al protocolo serie del
robot. En sentido contrario publica odometría, postura y telemetría AX-12A.

```bash
ros2 launch hexapod_bringup hexapod.launch.py
```

El paquete contiene el adaptador del hardware. Los
parámetros del nodo están en `config/esp32_bridge.yaml`. El nodo debe recibir
siempre este archivo; no contiene valores de configuración por defecto. La
interfaz está en `hexapod_web`.

La web es la unica que publica en `/hexapod/cmd_vel`: envia sus controles en
manual y reenvia las propuestas del evitador de obstaculos en autonomia.
El puente conserva sus limites y su watchdog, sin cambios.

Mientras existe movimiento, `cmd_vel` debe renovarse periódicamente. Si no se
recibe durante `cmd_vel_timeout_s`, el puente envía una orden de velocidad cero.

## Comandos serie importantes

- `MOD,n`: cambia modo de marcha.
- `GAT,periodo,altura`: ajusta el periodo y la altura de paso.
- `SMO,t`: cambia suavizado.
- `CMD,vx,vy,giro`: movimiento limitado.
- `POS,roll,pitch,yaw,x,y,z`: postura del cuerpo.
- `REG,id,address,size`: lectura de uno o dos bytes de un registro AX-12A.
- `WREG,id,address,size,value`: escritura de uno o dos bytes de un registro AX-12A.

La ESP32 publica continuamente la odometría y la postura mediante mensajes
`ODO`, con un periodo nominal de 1 s.

Las respuestas `AXR` y `AXW` se publican en el topic `hexapod/servo_register` y
se consultan desde la vista web de servos. La escritura no relee el registro:
la comprobacion se realiza manualmente con el boton `Leer`.
