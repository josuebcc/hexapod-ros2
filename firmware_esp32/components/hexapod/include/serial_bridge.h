/**
 * @file serial_bridge.h
 * @brief API del puente UART entre el ESP32 y ROS 2.
 * @ingroup group_serial_bridge
 */

#pragma once

#include "robot_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_serial_bridge serial_bridge
 * @brief Protocolo ASCII entre la capa de control y ROS 2.
 *
 * El módulo configura UART1 a 115200 bit/s y procesa mensajes terminados en
 * salto de línea. Los comandos recibidos actualizan atómicamente
 * @ref robot_controller_t; la locomoción y el diagnóstico los atienden desde
 * sus propias tareas.
 *
 * @par Comandos recibidos
 * - `CMD,vx,vy,vyaw`: velocidad lineal (mm/s) y angular (rad/s).
 * - `MOD,mode`: modo STOP, TRIPOD, RIPPLE o AMBLE, codificado de 0 a 3.
 * - `POS,roll,pitch,yaw,x,y,z`: postura objetivo, en radianes y milímetros.
 * - `GAT,period_s,step_h_mm`: periodo y altura máxima del paso.
 * - `SMO,time_s`: tiempo de transición de la postura.
 * - `REG,id,address,size` y `WREG,id,address,size,value`: acceso a registros.
 * - `RST`: reinicio del origen odométrico.
 * - `HELP` o `?`: descripción textual del protocolo.
 *
 * @par Tramas transmitidas
 * - `ODO`: odometría y postura, publicadas automáticamente cada segundo.
 * - `AXH`: última muestra de tensión, temperatura y carga de un servo.
 * - `AXR`: resultado de una lectura de registro AX-12A.
 * - `AXW`: resultado de una escritura de registro AX-12A.
 *
 * @par Código fuente
 * Declaración en serial_bridge.h e implementación en serial_bridge.cpp.
 *
 * @see group_robot_controller
 * @see group_diagnostic
 * @see group_locomotion_task
 * @{
 */

/* --------------------------------------------------------------------------
 * Inicialización
 * -------------------------------------------------------------------------- */

/** @name Inicialización
 *  @{ */

/**
 * @brief Inicia la tarea FreeRTOS para el puente serie en el núcleo 0.
 *
 * Configura UART1 sobre los GPIO 4 y 5 y crea una tarea en el núcleo 0 con
 * prioridad 4 y una pila de 4096 bytes.
 *
 * @param[in] rc Controlador compartido del robot.
 *
 * @pre El controlador debe estar inicializado y conservar su validez durante
 *      toda la ejecución de la tarea.
 */
void serial_bridge_start(robot_controller_t *rc);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
