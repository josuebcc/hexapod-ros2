/**
 * @file locomotion_task.h
 * @brief Tarea principal de locomoción y orquestación del pipeline de control.
 * @ingroup group_locomotion_task
 *
 * @details
 * Este módulo implementa el **bucle de control principal** del hexápodo,
 * ejecutado como una tarea FreeRTOS en el núcleo 1 a 50 Hz.
 *
 * En cada ciclo, la tarea ejecuta el siguiente pipeline en orden:
 *
 * @par Pipeline de control
 * 1. **Postura**: `posture_engine` suaviza los cambios de roll/pitch/yaw/x/y/z.
 * 2. **Cinemática del cuerpo**: actualiza `T_body` en `body_kinematics`.
 * 3. **Motor de marcha**: `gait_engine` genera las posiciones `foot_body[6]`.
 * 4. **Cinemática inversa**: `leg_compute_deg` calcula los ángulos de servo por pata.
 * 5. **Confirmación**: solo confirma el estado provisional si las seis IK son válidas.
 * 6. **Odometría**: extrae y publica la pose acumulada en `robot_controller`.
 * 7. **Servos**: `ax12_sync_move_deg_speed` envía los 18 ángulos en un paquete Sync Write.
 *
 * Al cambiar entre dos patrones activos, la tarea mantiene el patrón anterior
 * durante dos periodos con objetivo de movimiento nulo. Después configura el
 * nuevo modo, que reinicia su fase. `GAIT_MODE_STOP` también solicita
 * movimiento nulo, pero conserva el último patrón activo.
 *
 * La lectura de telemetría y el acceso bajo demanda a registros se ejecutan
 * en una segunda tarea de menor prioridad. Ambas tareas serializan el acceso
 * al bus DYNAMIXEL mediante un mutex.
 *
 * @par Código fuente
 * Declaración en locomotion_task.h e implementación en locomotion_task.cpp.
 *
 * @see group_gait_engine
 * @see group_body_kinematics
 * @see group_leg_kinematics
 * @see group_diagnostic
 */

/**
 * @defgroup group_locomotion_task locomotion_task
 * @brief Orquestación periódica del pipeline de locomoción.
 * @{
 */

#pragma once

#include "robot_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * API pública
 * -------------------------------------------------------------------------- */

/**
 * @brief Arranca la tarea de locomoción y la fija en el núcleo 1.
 *
 * Realiza las siguientes operaciones en orden:
 * 1. Crea la tarea FreeRTOS a 50 Hz, con prioridad 10 y 8192 bytes de stack.
 * 2. La tarea inicializa UART2 y el bus AX-12A desde el núcleo 1, configura
 *    los actuadores y activa su par.
 * 3. Inicializa `body_kinematics`, `posture_engine` y `gait_engine`.
 * 4. Entra en el bucle de control periódico.
 *
 * El contexto y los parámetros de la tarea son estáticos; esta función debe
 * llamarse una sola vez.
 *
 * @param[in] rc Controlador compartido, inicializado previamente mediante en robot_controller_init.
 * 
 */
void locomotion_task_start(robot_controller_t *rc);

/** @} */

#ifdef __cplusplus
}
#endif
