/**
 * @file robot_controller.h
 * @brief API del estado compartido y sincronizado del robot.
 * @ingroup group_robot_controller
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "leg_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_robot_controller robot_controller
 * @brief Intercambio atómico de comandos y resultados entre tareas.
 *
 * El módulo mantiene la copia canónica de las consignas de marcha y postura,
 * las salidas publicadas por locomotion_task y los datos intercambiados con la
 * tarea de diagnóstico. Un mutex estático protege todas las lecturas y
 * escrituras, de modo que cada consumidor obtiene una instantánea coherente
 * sin mantener el bloqueo durante su procesamiento.
 *
 * @par Código fuente
 * Declaración en robot_controller.h e implementación en robot_controller.cpp.
 *
 * @see group_locomotion_task
 * @see group_diagnostic
 * @see group_serial_bridge
 * @{
 */

/* --------------------------------------------------------------------------
 * Tipos públicos
 * -------------------------------------------------------------------------- */

/** @name Tipos públicos
 *  @{ */

/**
 * @brief Estado completo del robot en un instante dado.
 *
 * Agrupa consignas, configuración, resultados de locomoción y el buzón de
 * diagnóstico. Se copia por valor en `robot_get_state()` para garantizar una
 * lectura atómica sin mantener el mutex durante el procesamiento.
 *
 * @note Las unidades son: mm para distancias, mm/s para velocidades lineales,
 *       rad o rad/s para ángulos y velocidades angulares, y segundos para tiempos.
 */
typedef struct {
    /* Estado general. */
    gait_mode_t mode; /**< Modo de marcha solicitado. */

    /* Velocidad comandada. */
    float v_x;            /**< Velocidad de avance longitudinal (mm/s). */
    float v_y;            /**< Velocidad de desplazamiento lateral (mm/s). */
    float v_yaw;          /**< Velocidad de giro en Z (rad/s). */

    /* Configuración de la marcha. */
    float period_s;       /**< Duración de un ciclo completo de paso (segundos). */
    float step_h;         /**< Altura máxima de elevación de pata en fase de vuelo (mm). */

    /* Postura objetivo del cuerpo. */
    float roll;           /**< Rotación del chasis alrededor del eje X (rad). */
    float pitch;          /**< Rotación del chasis alrededor del eje Y (rad). */
    float yaw;            /**< Rotación local del chasis alrededor del eje Z (rad). */
    float x;              /**< Desplazamiento longitudinal del chasis respecto al centro (mm). */
    float y;              /**< Desplazamiento lateral del chasis respecto al centro (mm). */
    float z;              /**< Traslación vertical respecto a la posición nominal (mm). */
    float transition_time_s; /**< Tiempo de suavizado para cambios de postura (segundos). */

    /* Odometría publicada por locomotion_task. */
    float odo_x_mm;       /**< Posición acumulada en X desde el último reset (mm). */
    float odo_y_mm;       /**< Posición acumulada en Y desde el último reset (mm). */
    float odo_yaw_rad;    /**< Orientación acumulada en Z desde el último reset (rad). */

    /* Postura suavizada publicada por locomotion_task. */
    float curr_roll;      /**< Roll actual suavizado (rad). */
    float curr_pitch;     /**< Pitch actual suavizado (rad). */
    float curr_yaw;       /**< Yaw actual suavizado (rad). */
    float curr_x;         /**< Desplazamiento X actual suavizado (mm). */
    float curr_y;         /**< Desplazamiento Y actual suavizado (mm). */
    float curr_z;         /**< Altura actual suavizada (mm). */

    /* Última telemetría de un servo AX-12A. */
    uint32_t servo_health_seq;   /**< Incrementa en cada nueva muestra de health. */
    uint8_t servo_health_id;     /**< Último ID de servo actualizado. */
    uint8_t servo_voltage_raw;   /**< Present Voltage (décimas de voltio). */
    uint8_t servo_temperature_c; /**< Present Temperature (°C). */
    uint16_t servo_load_raw;     /**< Present Load RAW. */
    uint8_t servo_status_error_raw; /**< Byte de error del último status packet. */
    bool servo_health_valid;     /**< true si la última muestra es válida. */

    /* Petición y resultado de registros AX-12A bajo demanda. */
    uint32_t servo_register_request_seq; /**< Incrementa con cada nueva petición. */
    uint8_t servo_register_request_id;   /**< ID solicitado. */
    uint8_t servo_register_request_address; /**< Dirección solicitada. */
    uint8_t servo_register_request_size; /**< Número de bytes solicitado. */
    bool servo_register_request_write;   /**< true para escritura, false para lectura. */
    uint16_t servo_register_request_value; /**< Valor RAW solicitado en escritura. */
    uint32_t servo_register_result_seq;  /**< Incrementa con cada respuesta. */
    uint8_t servo_register_result_id;    /**< ID de la última respuesta. */
    uint8_t servo_register_result_address; /**< Dirección de la última respuesta. */
    uint8_t servo_register_result_size;  /**< Tamaño de la última respuesta. */
    uint16_t servo_register_result_value; /**< Valor RAW leído o solicitado en escritura. */
    uint8_t servo_register_result_error; /**< Campo Error del Status Packet. */
    bool servo_register_result_valid;    /**< true si la operación fue aceptada. */
    bool servo_register_result_write;    /**< true si la respuesta corresponde a una escritura. */

    /* Solicitudes de control consumibles. */
    bool reset_odometry_req; /**< Flag de solicitud de reset de odometría. */
} robot_state_t;

/**
 * @brief Contexto del controlador del robot.
 *
 * Contiene el estado compartido y el mutex estático que lo protege.
 */
typedef struct {
    StaticSemaphore_t mutex_buf; /**< Buffer estático para el mutex (sin malloc). */
    SemaphoreHandle_t mutex;     /**< Handle del mutex FreeRTOS. */
    robot_state_t st;            /**< Estado interno protegido por mutex. */
} robot_controller_t;

/** @} */

/* --------------------------------------------------------------------------
 * Inicialización
 * -------------------------------------------------------------------------- */

/** @name Inicialización
 *  @{ */

/**
 * @brief Inicializa el controlador del robot con valores por defecto seguros.
 *
 * Debe llamarse **una sola vez** antes de arrancar cualquier tarea.
 * Crea el mutex estático e inicializa el estado con valores conservadores.
 *
 * @param[out] rc Puntero al controlador a inicializar. Debe tener vida estática.
 */
void robot_controller_init(robot_controller_t *rc);

/** @} */

/* --------------------------------------------------------------------------
 * Comandos
 * -------------------------------------------------------------------------- */

/** @name Comandos
 *  @{ */

/**
 * @brief Establece el modo de marcha activo.
 *
 * Los valores fuera de @ref gait_mode_t se sustituyen por
 * `GAIT_MODE_STOP`.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] mode Nuevo modo de marcha.
 */
void robot_set_mode(robot_controller_t *rc, gait_mode_t mode);

/**
 * @brief Establece el vector de velocidad de desplazamiento.
 *
 * Esta función únicamente almacena el comando. La tarea de locomoción lo
 * convierte en desplazamiento por ciclo y @ref gait_engine_set_command aplica
 * el límite duro del paso efectivo antes de generar las trayectorias.
 * Si alguna componente no es finita, las tres velocidades se sustituyen por
 * cero para evitar propagar valores inválidos a la cinemática.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] vx Velocidad longitudinal, en mm/s. Positivo hacia delante.
 * @param[in] vy Velocidad lateral, en mm/s. Positivo hacia la izquierda.
 * @param[in] vyaw_rad Velocidad angular alrededor de Z, en rad/s. Positivo en
 *                     sentido antihorario.
 */
void robot_set_velocity(robot_controller_t *rc, float vx, float vy, float vyaw_rad);

/**
 * @brief Ajusta los parámetros dinámicos de la marcha.
 *
 * Permite modificar el periodo y la altura de paso en tiempo real. La tarea de
 * locomoción aplica los cambios directamente si no hay una transición de modo.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] period_s Duración de un ciclo completo, en s. Mínimo: 0,6 s.
 * @param[in] step_h_mm Altura máxima de elevación de pata, en mm. Intervalo
 *                      admitido: 10--60 mm.
 *
 * @note Los periodos positivos inferiores a 0,6 s se saturan a 0,6 s. Los
 *       valores positivos de altura se saturan al intervalo 10--60 mm. Los
 *       valores no positivos o no finitos se ignoran.
 */
void robot_set_gait_params(robot_controller_t *rc, float period_s, float step_h_mm);

/**
 * @brief Establece la postura estática del chasis (Body IK).
 *
 * Los ángulos y traslaciones se aplican al frame local del cuerpo
 * a través de `T_body` en `body_kinematics`. El suavizado es gestionado
 * por `posture_engine`. Si alguna componente no es finita, se descarta la
 * postura completa.
 *
 * Los límites se aplican individualmente a cada componente. La validación
 * conjunta de la postura y la marcha corresponde a la cinemática inversa de
 * las seis patas en `locomotion_task`.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] roll Rotación en X, en rad. Positivo: lado derecho hacia arriba.
 * @param[in] pitch Rotación en Y, en rad. Positivo: frente hacia arriba.
 * @param[in] yaw Rotación local en Z, en rad.
 * @param[in] x Desplazamiento en X, en mm.
 * @param[in] ty Desplazamiento en Y, en mm.
 * @param[in] z Variación de altura, en mm. Positivo hacia arriba.
 */
void robot_set_posture(robot_controller_t *rc, float roll, float pitch, float yaw, float x, float ty, float z);

/**
 * @brief Configura el tiempo de transición para suavizado de postura.
 *
 * Controla la velocidad de los cambios de postura a través de `posture_engine`.
 * Un valor mayor genera movimientos más lentos y suaves.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] time_s Duración de la transición, en s. Los valores positivos
 *                   inferiores a 0,1 s se saturan a 0,1 s; los valores no
 *                   positivos o no finitos se ignoran.
 */
void robot_set_smooth_time(robot_controller_t *rc, float time_s);

/** @} */

/* --------------------------------------------------------------------------
 * Lectura del estado
 * -------------------------------------------------------------------------- */

/** @name Lectura del estado
 *  @{ */

/**
 * @brief Obtiene una copia atómica del estado actual del robot.
 *
 * Adquiere el mutex, copia `robot_state_t` y lo libera inmediatamente. El
 * llamador puede procesar la instantánea sin bloquear las demás tareas.
 *
 * @param[in,out] rc Controlador compartido.
 * @return Copia coherente del estado en el momento de la llamada.
 */
robot_state_t robot_get_state(robot_controller_t *rc);

/** @} */

/* --------------------------------------------------------------------------
 * Publicación interna
 * -------------------------------------------------------------------------- */

/** @name Publicación interna
 *  @{ */

/**
 * @brief Actualiza los datos de odometría reportados por la tarea de locomoción.
 *
 * Esta función es llamada internamente por `locomotion_task` a 50 Hz
 * para actualizar la posición estimada del robot. **No debe llamarse
 * desde código de usuario.**
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] x Posición acumulada en X, en mm.
 * @param[in] y Posición acumulada en Y, en mm.
 * @param[in] yaw Orientación acumulada en Z, en rad.
 * @param[in] roll Roll actual suavizado del cuerpo, en rad.
 * @param[in] pitch Pitch actual suavizado del cuerpo, en rad.
 * @param[in] body_yaw Yaw local actual suavizado del cuerpo, en rad.
 * @param[in] bx Desplazamiento X actual suavizado del cuerpo, en mm.
 * @param[in] by Desplazamiento Y actual suavizado del cuerpo, en mm.
 * @param[in] bz Desplazamiento Z actual suavizado del cuerpo, en mm.
 */
void robot_report_odometry(robot_controller_t *rc, float x, float y, float yaw, float roll, float pitch, float body_yaw, float bx, float by, float bz);

/**
 * @brief Publica la última muestra de health de un servo AX-12A.
 *
 * Esta función es llamada internamente por la tarea de diagnóstico.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] id ID del servo consultado.
 * @param[in] voltage_raw Tensión en décimas de voltio.
 * @param[in] temperature_c Temperatura, en grados Celsius.
 * @param[in] load_raw Carga en formato RAW del AX-12A.
 * @param[in] status_error_raw Byte Error del Status Packet.
 * @param[in] valid true si se recibió una respuesta válida.
 */
void robot_report_servo_health(robot_controller_t *rc, uint8_t id, uint8_t voltage_raw, uint8_t temperature_c, uint16_t load_raw, uint8_t status_error_raw, bool valid);

/** @} */

/* --------------------------------------------------------------------------
 * Acceso a registros AX-12A
 * -------------------------------------------------------------------------- */

/** @name Acceso a registros AX-12A
 *  @{ */

/**
 * @brief Solicita una lectura de registro a la tarea de diagnóstico.
 *
 * Una petición nueva sustituye a otra que todavía no haya sido recogida.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] id ID del servo.
 * @param[in] address Dirección inicial del registro.
 * @param[in] size Tamaño de lectura, en bytes.
 */
void robot_request_servo_register(robot_controller_t *rc, uint8_t id, uint8_t address, uint8_t size);

/**
 * @brief Solicita una escritura de registro a la tarea de diagnóstico.
 *
 * Una petición nueva sustituye a otra que todavía no haya sido recogida.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] id ID del servo.
 * @param[in] address Dirección inicial del registro.
 * @param[in] size Tamaño de escritura, en bytes.
 * @param[in] value Valor RAW que se escribirá.
 */
void robot_request_servo_register_write(robot_controller_t *rc, uint8_t id, uint8_t address, uint8_t size, uint16_t value);

/**
 * @brief Publica el resultado de una lectura de registro AX-12A.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] id ID del servo consultado.
 * @param[in] address Dirección inicial del registro.
 * @param[in] size Tamaño solicitado, en bytes.
 * @param[in] value Valor RAW leído.
 * @param[in] status_error_raw Byte Error del Status Packet.
 * @param[in] valid true si se recibió una respuesta válida.
 */
void robot_report_servo_register(robot_controller_t *rc, uint8_t id, uint8_t address, uint8_t size, uint16_t value, uint8_t status_error_raw, bool valid);

/**
 * @brief Publica el resultado de una escritura de registro AX-12A.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] id ID del servo.
 * @param[in] address Dirección inicial del registro.
 * @param[in] size Tamaño solicitado, en bytes.
 * @param[in] value Valor RAW solicitado.
 * @param[in] status_error_raw Byte Error del Status Packet.
 * @param[in] valid true si la transacción finalizó correctamente.
 */
void robot_report_servo_register_write(robot_controller_t *rc, uint8_t id, uint8_t address, uint8_t size, uint16_t value, uint8_t status_error_raw, bool valid);

/** @} */

/* --------------------------------------------------------------------------
 * Reset de odometría
 * -------------------------------------------------------------------------- */

/** @name Reset de odometría
 *  @{ */

/**
 * @brief Solicita un reset de la odometría acumulada.
 *
 * Activa el flag `reset_odometry_req` de forma thread-safe.
 * La tarea de locomoción consumirá atómicamente la solicitud en su próximo
 * ciclo.
 *
 * @param[in,out] rc Controlador compartido.
 */
void robot_reset_odometry(robot_controller_t *rc);

/**
 * @brief Consume atómicamente una solicitud de reset de odometría.
 *
 * Lee y limpia `reset_odometry_req` bajo el mutex del controlador. Esta función
 * está destinada a la tarea de locomoción, que es la propietaria del estado
 * cinemático.
 *
 * @param[in,out] rc Controlador compartido.
 * @return true si había una solicitud pendiente; false en caso contrario.
 */
bool robot_consume_odometry_reset(robot_controller_t *rc);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
