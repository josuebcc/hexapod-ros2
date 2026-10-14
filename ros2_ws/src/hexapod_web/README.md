# hexapod_web

Panel Flask integrado en ROS 2. Publica comandos para el puente serie y muestra
la odometría estimada, la postura y la telemetría AX-12A.

El ejecutable y el nodo ROS 2 se llaman `web_panel`.

El paquete ofrece supervision, control manual y seleccion de autonomia.
Sus parámetros están en `config/web_panel.yaml`.
El panel renueva la velocidad a la frecuencia `motion.cmd_vel_rate_hz`; el
puente puede detectar así una pérdida de comunicación con el navegador.

La web es la unica que publica velocidades en `/hexapod/cmd_vel`.
En manual envia sus controles; en autonomia reenvia cada propuesta nueva de
`/obstacle_avoider/cmd_vel`. El interruptor cambia una variable en el servidor web:
no hay servicio ROS de activacion ni cambios de modo en el evitador.
El panel bloquea las entradas manuales en autonomia. Al cambiar de modo se
ponen a cero las entradas locales y se publica una parada. Stop (`MOD,0`)
tambien devuelve la web a manual.
Las rutas `GET /control_mode` y `POST /control_mode` consultan y solicitan el
modo (`manual` o `autonomous`). Activar autonomia requiere una propuesta valida
recibida hace menos de 0.5 s; manual funciona aunque el evitador no este arrancado.
El servidor rechaza velocidades manuales durante la autonomia (HTTP 409).
Cerrar el panel no cancela el modo autonomo.
Reiniciar el nodo web vuelve a manual. Nunca se reenvia una propuesta guardada
al cambiar de modo ni se repite la ultima: si dejan de llegar propuestas,
el watchdog del puente detiene el robot (`cmd_vel_timeout_s`, 1 s por defecto).

El nodo no contiene valores por defecto: debe arrancarse mediante
`hexapod_bringup`, que carga `config/web_panel.yaml`.

```bash
ros2 launch hexapod_bringup hexapod.launch.py
```

La interfaz queda disponible, por defecto, en `http://localhost:8080`.

## Estructura

- `web_server.py`: arranca ROS 2 y Flask.
- `web_app.py`: define las rutas HTTP.
- `web_node.py`: contiene las interfaces ROS 2 y el estado del panel.
- `servo_registers.py`: aísla el catálogo y las operaciones AX-12A.
- `payloads.py`: crea los estados iniciales de la API.
- `static/js/panel/`: lógica del panel separada por función.
- `static/js/servos/`: diagnóstico y registros de los servos.
- `static/js/shared/`: utilidades compartidas de interfaz.
- `static/css/base.css`: tema y componentes comunes.
- `static/css/panel.css`: distribución del panel principal.
- `static/css/controls.css`: controles de movimiento, marcha y postura.
- `static/css/servos.css`: diagrama y tabla AX-12A.

Las plantillas cargan `panel/main.js` o `servos/main.js`; estos son los únicos
puntos de entrada JavaScript de cada página.
