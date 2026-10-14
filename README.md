# hexapod-ros2

Repositorio oficial asociado al Trabajo Fin de Grado **Diseño e implementación
del motor cinemático y navegación en ROS 2 de un robot hexápodo**.

Reúne el firmware del ESP32, los paquetes de ROS 2, las fuentes de la memoria,
la documentación Doxygen y los archivos de diseño y fabricación de la PCB
principal.

## Estructura

```text
firmware_esp32/          Firmware de bajo nivel desarrollado con ESP-IDF
ros2_ws/                 Espacio de trabajo y paquetes de ROS 2
documentacion_doxygen/   Configuración y recursos de documentación de la API
memoria_TFG/             Fuentes LaTeX y PDF final de la memoria
hardware/                Diseños mecánicos y electrónicos
└── pcb/
    ├── fuentes/        Proyecto y librerías de Altium Designer
    ├── fabricacion/    Archivos destinados a fabricar la PCB
    └── documentacion/ Documentos exportados del diseño
```

Cada subsistema incluye instrucciones específicas en su propio `README.md`.

## Licencia

Distribuido bajo la licencia MIT. Consulta el archivo [LICENSE](LICENSE).
