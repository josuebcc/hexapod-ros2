# Firmware de bajo nivel del hexápodo

Firmware ESP-IDF para el control de un hexápodo PhantomX Mark III mediante un
ESP32 y 18 servomotores DYNAMIXEL AX-12A. El programa genera las marchas,
resuelve la cinemática, transmite las consignas articulares y ofrece una
interfaz de comandos y telemetría por UART.

## Requisitos

- ESP-IDF 6.1.0 y un ESP32.
- PCB con la interfaz *half-duplex* para el bus DYNAMIXEL.
- 18 AX-12A configurados a 1 Mbit/s y con IDs del 1 al 18.
- Alimentación adecuada para la lógica y los actuadores.
- Cable USB para programar y, para una prueba directa, un adaptador USB--UART
  de 3,3 V.

> [!WARNING]
> Realiza la primera puesta en marcha con el hexápodo elevado y las patas
> completamente libres. Al arrancar, el firmware activa el par de los AX-12A y
> envía la postura neutral aunque el modo inicial sea `STOP`. Haz todas las
> conexiones con la alimentación de los actuadores desconectada.

## Distribución de los servos

| Pata | Coxa | Fémur | Tibia |
|---|---:|---:|---:|
| Izquierda delantera | 1 | 3 | 5 |
| Izquierda media | 13 | 15 | 17 |
| Izquierda trasera | 7 | 9 | 11 |
| Derecha trasera | 8 | 10 | 12 |
| Derecha media | 14 | 16 | 18 |
| Derecha delantera | 2 | 4 | 6 |

## Conexiones

| Interfaz | Señales | Velocidad | Uso |
|---|---|---:|---|
| UART0 | USB/programación | 115200 bit/s | Carga y mensajes de depuración |
| UART1 | TX: GPIO4, RX: GPIO5 | 115200 bit/s | Comandos y telemetría |
| UART2 | TX: GPIO16, RX: GPIO17, DIR: GPIO2 | 1 Mbit/s | Bus *half-duplex* de los AX-12A |

Para controlar el firmware directamente, conecta el RX de un adaptador
USB--UART de 3,3 V al GPIO4, su TX al GPIO5 y une las masas. No apliques una
señal serie de 5 V al ESP32. Los comandos se envían por UART1 como líneas de
texto terminadas en `LF`; la consola USB de programación utiliza UART0 y no
recibe estas órdenes.

## Compilación y carga

Abre una terminal con el entorno de ESP-IDF inicializado y ejecuta:

```bash
cd /ruta/al/proyecto/firmware_esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Adapta el puerto si la placa aparece, por ejemplo, como `/dev/ttyACM0`. El
monitor se cierra con `Ctrl+]`.

El firmware puede cargarse con los actuadores sin alimentar. Para la puesta en
marcha real, alimenta el bus y reinicia el ESP32: el escaneo, la configuración
de los AX-12A y la activación del par solo se realizan durante el arranque.

## Primera puesta en marcha

1. Eleva el robot y deja las seis patas libres.
2. Con la potencia desconectada, revisa polaridad, masa común, bus y ramales.
3. Conecta UART0 para observar el arranque y, si no utilizas ROS 2, un terminal
   independiente a UART1 a 115200 bit/s, 8N1.
4. Alimenta el sistema o reinicia el ESP32 con los AX-12A disponibles.
5. Comprueba que el escaneo termina con `18/18 respuestas`.
6. Verifica que las patas alcanzan la postura neutral sin colisiones,
   vibraciones ni giros incorrectos.

El estado inicial es `STOP`, con periodo de 2 s, altura de paso de 40 mm,
postura local nula y tiempo de suavizado de 1 s. `STOP` evita el avance, pero no
desactiva el par.

## Prueba funcional

Con el robot todavía elevado, envía por UART1:

```text
HELP
MOD,0
CMD,0,0,0
POS,0,0,0,0,0,0
SMO,2
GAT,2,20
MOD,1
CMD,20,0,0
```

Esta secuencia selecciona una transición suave, reduce la altura del paso y
prueba un avance pequeño en modo trípode. Para detenerlo, envía `CMD,0,0,0`,
espera a que cese el movimiento y después envía `MOD,0`.

Las velocidades permanecen almacenadas hasta recibir otro comando o reiniciar
el sistema. No desconectes el enlace de control mientras haya una consigna de
velocidad distinta de cero.

## Comandos de UART1

| Comando | Función |
|---|---|
| `HELP` o `?` | Muestra la ayuda integrada |
| `MOD,m` | Selecciona `0=STOP`, `1=TRIPOD`, `2=RIPPLE` o `3=AMBLE` |
| `CMD,vx,vy,vyaw` | Velocidades X/Y en mm/s y giro en rad/s |
| `POS,roll,pitch,yaw,x,y,z` | Postura; rotaciones en radianes y desplazamientos en milímetros |
| `GAT,period_s,step_h_mm` | Periodo y altura del paso |
| `SMO,time_s` | Tiempo de suavizado de la postura |
| `RST` | Reinicia el origen de la odometría |
| `REG,id,address,size` | Lee uno o dos bytes de un registro AX-12A |
| `WREG,id,address,size,value` | Escribe uno o dos bytes de un registro AX-12A |

Límites principales:

- Postura: *roll* y *pitch* `+-0.35 rad`, *yaw* `+-0.52 rad`, X/Y `+-40 mm`
  y Z `+-50 mm`.
- Marcha: periodo mínimo `0.6 s` y altura entre `10` y `60 mm`.
- Paso efectivo combinado: `120 mm` en X, `50 mm` en Y y `0.35 rad` de giro.

El generador reduce proporcionalmente las componentes de una orden que supere
el límite combinado. Aun así, empieza con valores pequeños: una combinación
extrema de postura y marcha puede quedar fuera del alcance cinemático.

> [!CAUTION]
> `WREG` permite modificar registros de EEPROM, incluidos el ID y la velocidad
> del bus. Resérvalo para diagnóstico y utilízalo solo si conoces la tabla de
> control del AX-12A.

## Telemetría

| Mensaje | Contenido |
|---|---|
| `ODO,x,y,yaw,roll,pitch,body_yaw,x_body,y_body,z_body` | Odometría plana y postura local, una vez por segundo |
| `AXH,id,valid,V,T,load,error` | Estado del actuador; `V` está en décimas de voltio |
| `AXR,id,address,size,valid,value,error` | Resultado de una lectura `REG` |
| `AXW,id,address,size,valid,value,error` | Resultado de una escritura `WREG` |

Una respuesta `AXH` no válida o con un byte de error distinto de cero debe
revisarse antes de apoyar el robot. La odometría es una estimación calculada a
partir de la consigna de marcha, no una medida externa del desplazamiento real.

## Parada y problemas frecuentes

La parada ordinaria consiste en enviar `CMD,0,0,0`, esperar a que termine la
transición y seleccionar `MOD,0`. Esta orden mantiene el par. Desconecta la
alimentación de los actuadores antes de manipular las articulaciones. Si se
pierde la comunicación durante el movimiento, corta dicha alimentación.

- **No se detecta ningún servo:** comprueba alimentación, masa común, interfaz
  *half-duplex*, 1 Mbit/s y los GPIO16, GPIO17 y GPIO2.
- **El escaneo es incompleto:** revisa el ID, el cableado y el ramal indicado;
  no puede haber IDs repetidos.
- **No llegan comandos:** comprueba que utilizas UART1, TX/RX cruzados,
  115200 bit/s, 8N1 y terminación `LF`.
- **Aparece una IK inválida:** envía velocidad cero, recupera la postura
  neutral y reduce la altura, la velocidad o el desplazamiento.
- **Una articulación gira al revés:** detén el robot y revisa IDs, signos y
  ajustes de montaje en `components/hexapod/leg_config.cpp`.
