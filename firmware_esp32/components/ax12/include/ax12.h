/**
 * @file ax12.h
 * @brief API del controlador de servomotores DYNAMIXEL AX-12A.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_ax12 ax12
 * @brief Control de los servomotores y acceso al bus DYNAMIXEL.
 *
 * Este es el único módulo que debe utilizar el resto del firmware para acceder
 * a los actuadores. Ofrece una API de alto nivel orientada al hexápodo sobre el
 * port a ESP32 del protocolo DYNAMIXEL desarrollado por Hans (hansj66).
 * El módulo administra una única instancia del bus DYNAMIXEL 1.0 conectado a
 * UART2. La API está limitada a los 18 actuadores del robot, cuyos IDs válidos
 * son 1--18.
 *
 * La secuencia normal de uso es:
 * @code
 * ax12_init(1000000);
 * ax12_scan();
 * ax12_configure_eeprom_ram();
 * ax12_set_torque(id, true);
 * @endcode
 *
 * @par Código fuente
 * Declaración en ax12.h e implementación en ax12.cpp.
 *
 * @see Port del protocolo DYNAMIXEL: https://github.com/hansj66/dynamixel2espressif
 *
 * @note El módulo serializa internamente todas las operaciones mediante un
 *       mutex, por lo que puede utilizarse desde varias tareas FreeRTOS.
 * @note Las operaciones que reciben `timeout_ms` pueden bloquear como máximo
 *       durante el tiempo indicado por ese parámetro.
 * @{
 */

/* --------------------------------------------------------------------------
 * Tipos públicos
 * -------------------------------------------------------------------------- */

/** @name Tipos públicos
 *  @{ */

/** @brief Muestra de telemetría dinámica de un AX-12A. */
typedef struct {
    uint8_t voltage_raw;       /**< Tensión en décimas de voltio. */
    uint8_t temperature_c;     /**< Temperatura interna en grados Celsius. */
    uint16_t load_raw;         /**< Carga AX-12A; bits 0--9: magnitud, bit 10: sentido. */
    uint8_t status_error_raw;  /**< Campo Error del Status Packet recibido. */
} ax12_health_t;

/** @brief Resultado de la lectura de un registro de la tabla de control. */
typedef struct {
    uint16_t value;            /**< Valor RAW de uno o dos bytes. */
    uint8_t status_error_raw;  /**< Campo Error del Status Packet recibido. */
} ax12_register_read_t;

/** @brief Resultado de la escritura de un registro de la tabla de control. */
typedef struct {
    uint8_t status_error_raw;  /**< Campo Error del Status Packet recibido. */
} ax12_register_write_t;

/** @} */

/* --------------------------------------------------------------------------
 * Inicialización y diagnóstico
 * -------------------------------------------------------------------------- */

/** @name Inicialización y diagnóstico
 *  @{ */

/**
 * @brief Inicializa el bus DYNAMIXEL.
 *
 * Configura UART2 con la velocidad indicada y selecciona el protocolo
 * DYNAMIXEL 1.0. También crea el mutex estático que protege las operaciones
 * posteriores sobre el bus.
 *
 * @param[in] baud Velocidad del bus en bits por segundo.
 * @note Debe llamarse una sola vez antes de utilizar el resto de la API.
 */
void ax12_init(int baud);

/**
 * @brief Envía un Ping a los IDs 1--18.
 *
 * El resultado se muestra mediante el log y no bloquea las transmisiones
 * posteriores.
 */
void ax12_scan(void);

/** @} */

/* --------------------------------------------------------------------------
 * Configuración de los actuadores
 * -------------------------------------------------------------------------- */

/** @name Configuración de los actuadores
 *  @{ */

/**
 * @brief Configura la EEPROM y la RAM de los IDs 1--18.
 *
 * Configura mediante Sync Write los siguientes registros de EEPROM:
 * - Return Delay Time: 250, equivalente a 500 us.
 * - Alarm LED: tensión, sobretemperatura y sobrecarga (0x25).
 * - Shutdown: apagado automático solo por sobretemperatura (0x04).
 *
 * A continuación configura en RAM los márgenes de compliance CW/CCW con valor
 * 4 y las pendientes CW/CCW con valor 64.
 */
void ax12_configure_eeprom_ram(void);

/** @} */

/* --------------------------------------------------------------------------
 * Control de par y movimiento
 * -------------------------------------------------------------------------- */

/** @name Control de par y movimiento
 *  @{ */

/**
 * @brief Activa o desactiva el par de un servo.
 *
 * @param[in] id Identificador del servo, entre 1 y 18.
 * @param[in] enabled true para activar el par y false para desactivarlo.
 * @return true si la operación fue aceptada; false si el ID es inválido o la
 *         comunicación falló.
 */
bool ax12_set_torque(uint8_t id, bool enabled);

/**
 * @brief Envía posiciones y velocidades mediante Sync Write.
 *
 * Escribe conjuntamente Goal Position y Moving Speed desde la dirección 0x1E.
 * Los ángulos se convierten linealmente de 0--300 grados a 0--1023 y se
 * saturan al rango físico. La velocidad se limita a 1--1023; el valor cero se
 * sustituye por uno para evitar el modo de velocidad máxima sin control.
 *
 * La función rechaza punteros nulos, cantidades fuera de 1--18, IDs fuera del
 * rango 1--18, IDs repetidos y ángulos no finitos.
 *
 * @param[in] ids IDs de los actuadores que se moverán.
 * @param[in] angles Posiciones objetivo en grados.
 * @param[in] count Número de elementos de @p ids y @p angles.
 * @param[in] speed Velocidad RAW común para todos los actuadores.
 * @return true si el paquete se construyó y transmitió localmente; false si
 *         los parámetros son inválidos o la transmisión no pudo iniciarse.
 * @note Sync Write no genera Status Packets y, por tanto, no confirma la
 *       recepción individual de los actuadores.
 */
bool ax12_sync_move_deg_speed(const uint8_t *ids,
                              const float *angles,
                              int count,
                              uint16_t speed);

/** @} */

/* --------------------------------------------------------------------------
 * Telemetría
 * -------------------------------------------------------------------------- */

/** @name Telemetría
 *  @{ */

/**
 * @brief Lee la telemetría dinámica de un AX-12A.
 *
 * Realiza una única lectura de los cuatro bytes consecutivos comprendidos
 * entre Present Load (0x28) y Present Temperature (0x2B). El campo Error se
 * obtiene del mismo Status Packet.
 *
 * @param[in] id Identificador del servo, entre 1 y 18.
 * @param[in] timeout_ms Tiempo máximo de espera de la transacción, en ms.
 * @param[out] out_health Estructura donde se almacena la muestra.
 * @return true si se recibieron los cuatro bytes; false si los argumentos son
 *         inválidos o la comunicación falló.
 */
bool ax12_read_health(uint8_t id,
                      uint32_t timeout_ms,
                      ax12_health_t *out_health);

/**
 * @brief Lee bajo demanda un registro de la tabla de control.
 *
 * @param[in] id Identificador del servo, entre 1 y 18.
 * @param[in] address Dirección inicial del registro, entre 0 y 49.
 * @param[in] size Tamaño del registro: uno o dos bytes.
 * @param[in] timeout_ms Tiempo máximo de espera de la transacción, en ms.
 * @param[out] out_read Valor RAW y campo Error del Status Packet.
 * @return true si se recibió el número de bytes solicitado.
 */
bool ax12_read_register(uint8_t id,
                        uint8_t address,
                        uint8_t size,
                        uint32_t timeout_ms,
                        ax12_register_read_t *out_read);

/**
 * @brief Escribe bajo demanda un registro de la tabla de control.
 *
 * Esta es una operación RAW: valida el ID, los límites de la tabla, el tamaño y
 * el ancho del valor, pero no decide qué registros puede modificar cada capa.
 * Si una interfaz necesita restringir registros, debe hacerlo antes de llamar
 * a esta función.
 *
 * @param[in] id Identificador del servo, entre 1 y 18.
 * @param[in] address Dirección inicial del registro.
 * @param[in] size Tamaño del registro: uno o dos bytes.
 * @param[in] value Valor RAW que se escribirá, en little-endian.
 * @param[in] timeout_ms Tiempo máximo de espera de la transacción, en ms.
 * @param[out] out_write Campo Error del Status Packet.
 * @return true si el servo aceptó la escritura; false si los argumentos son
 *         inválidos o la comunicación falló.
 * @note La función no comprueba si el registro es de solo lectura ni valida su
 *       rango semántico. Estas condiciones pueden ser indicadas por el servo en
 *       el campo Error del Status Packet.
 * @note La función no relee el registro después de escribirlo.
 */
bool ax12_write_register(uint8_t id,
                         uint8_t address,
                         uint8_t size,
                         uint16_t value,
                         uint32_t timeout_ms,
                         ax12_register_write_t *out_write);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
