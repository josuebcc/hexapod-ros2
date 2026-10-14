/**
 * @file diagnostic.h
 * @brief API de la supervisión auxiliar de los servomotores AX-12A.
 * @ingroup group_diagnostic
 */

#pragma once

#include <stdbool.h>
#include "robot_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_diagnostic diagnostic
 * @brief Telemetría y acceso bajo demanda a los registros de los servos.
 *
 * El módulo ejecuta las operaciones de diagnóstico en una tarea FreeRTOS de
 * menor prioridad que la locomoción. La tarea se despierta después de cada
 * ciclo de control y, si no hay una petición REG/WREG, consulta el siguiente
 * servo. A 50 Hz completa una ronda de los 18 actuadores cada 0,36 segundos.
 *
 * Las peticiones explícitas de registros tienen prioridad sobre la telemetría
 * periódica. Cada activación realiza como máximo una transacción AX-12A para
 * aprovechar el intervalo libre entre dos envíos de locomoción.
 *
 * @note El acceso concurrente al bus está serializado internamente por el
 *       módulo `ax12`.
 *
 * @par Código fuente
 * Declaración en diagnostic.h e implementación en diagnostic.cpp.
 *
 * @see group_ax12
 * @see group_robot_controller
 * @see group_locomotion_task
 * @{
 */

/* --------------------------------------------------------------------------
 * Inicialización
 * -------------------------------------------------------------------------- */

/** @name Inicialización
 *  @{ */

/**
 * @brief Inicia la tarea auxiliar de diagnóstico.
 *
 * La tarea se crea en el núcleo 1, con prioridad 9 y una pila de 4096 bytes.
 * Su contexto tiene almacenamiento estático, por lo que esta función debe
 * llamarse una sola vez.
 *
 * @param[in] rc Controlador compartido del robot.
 * @return true si la tarea se creó correctamente; false en caso contrario.
 *
 * @pre El controlador debe estar inicializado. El bus AX-12A debe estar listo
 *      antes de la primera llamada a `diagnostic_notify_cycle()`.
 */
bool diagnostic_start(robot_controller_t *rc);

/** @} */

/* --------------------------------------------------------------------------
 * Sincronización con locomotion_task
 * -------------------------------------------------------------------------- */

/** @name Sincronización con locomotion_task
 *  @{ */

/**
 * @brief Notifica que ha terminado un ciclo de locomoción.
 *
 * Incrementa la notificación directa de la tarea auxiliar y habilita una
 * oportunidad de trabajo. Si se acumulan varias mientras diagnóstico está
 * ocupado, se vacían conjuntamente y no se intentan recuperar las operaciones
 * omitidas.
 */
void diagnostic_notify_cycle(void);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
