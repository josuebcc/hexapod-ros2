/**
 * @file ax12.cpp
 * @brief Implementación del controlador de servomotores DYNAMIXEL AX-12A.
 * @ingroup group_ax12
 */

#include "ax12.h"

#include <math.h>
#include <string.h>

#include "dynamixel2espressif.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* --------------------------------------------------------------------------
 * Configuración general del bus
 * -------------------------------------------------------------------------- */

#define DXL_PROTOCOL_VERSION         1.0f
#define AX12_UART                    UART_NUM_2

#define AX12_MIN_SERVO_ID            1
#define AX12_MAX_SERVO_ID            18
#define AX12_SERVO_COUNT             18
#define AX12_CONTROL_TABLE_SIZE       50

/* Límites y unidades internas del AX-12A. */
#define AX12_MIN_ANGLE_DEG            0.0f
#define AX12_MAX_ANGLE_DEG            300.0f
#define AX12_MAX_POSITION_RAW         1023.0f
#define AX12_MAX_SPEED_RAW            1023

/* --------------------------------------------------------------------------
 * Direcciones de la tabla de control
 * -------------------------------------------------------------------------- */

#define AX12_ADDR_RETURN_DELAY       0x05
#define AX12_ADDR_ALARM_LED          0x11
#define AX12_ADDR_CW_MARGIN          0x1A
#define AX12_ADDR_GOAL_POSITION      0x1E
#define AX12_ADDR_PRESENT_LOAD       0x28

/* --------------------------------------------------------------------------
 * Configuración adoptada para los actuadores
 * -------------------------------------------------------------------------- */

#define AX12_RETURN_DELAY_VALUE      250
#define AX12_ALARM_LED_MASK          0x25
#define AX12_SHUTDOWN_MASK           0x04
#define AX12_COMPLIANCE_MARGIN       4
#define AX12_COMPLIANCE_SLOPE        64

#define AX12_PROTECTION_DATA_SIZE    2
#define AX12_COMPLIANCE_DATA_SIZE    4
#define AX12_MOVE_DATA_SIZE          4
#define AX12_HEALTH_DATA_SIZE        4
#define AX12_EEPROM_WRITE_DELAY_MS   20

/* --------------------------------------------------------------------------
 * Estado interno del módulo
 * -------------------------------------------------------------------------- */

static const char *TAG = "AX12";
static Dynamixel2Espressif s_dxl(AX12_UART, (gpio_num_t)CONFIG_DXL_DIR_PIN); /**< Instancia única del bus DYNAMIXEL. */
static StaticSemaphore_t s_bus_mutex_buffer; /**< Memoria estática del mutex del bus. */
static SemaphoreHandle_t s_bus_mutex;        /**< Exclusión mutua entre las tareas usuarias del bus. */

/* --------------------------------------------------------------------------
 * Funciones auxiliares
 * -------------------------------------------------------------------------- */


/**
 * @brief Comprueba si un ID de servo es válido (1--18).
 *
 * @param[in] id Identificador que se comprobará.
 * @return true si el ID pertenece al bus del robot; false en caso contrario.
 */
static bool is_valid_servo_id(uint8_t id)
{
    return id >= AX12_MIN_SERVO_ID && id <= AX12_MAX_SERVO_ID;
}

/**
 * @brief Adquiere el uso exclusivo del bus DYNAMIXEL.
 *
 * Antes de @ref ax12_init el mutex todavía no existe; esta condición permite
 * construir el estado interno sin utilizar memoria dinámica.
 */
static void lock_bus(void)
{
    if (s_bus_mutex) xSemaphoreTake(s_bus_mutex, portMAX_DELAY);
}

/**
 * @brief Libera el uso exclusivo del bus DYNAMIXEL.
 */
static void unlock_bus(void)
{
    if (s_bus_mutex) xSemaphoreGive(s_bus_mutex);
}

/* --------------------------------------------------------------------------
 * Inicialización y diagnóstico
 * -------------------------------------------------------------------------- */

void ax12_init(int baud)
{
    if (!s_bus_mutex) s_bus_mutex = xSemaphoreCreateMutexStatic(&s_bus_mutex_buffer);
    lock_bus();
    ESP_LOGI(TAG, "Inicializando AX-12A (UART%d, baud=%d)", AX12_UART, baud);
    s_dxl.begin(baud);
    s_dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    ESP_LOGI(TAG, "Bus DYNAMIXEL listo");
    unlock_bus();
}

void ax12_scan(void)
{
    uint32_t detected_mask = 0;
    uint8_t detected_count = 0;

    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_LOGI(TAG, "Escaneando AX-12A (IDs 1-18)...");

    lock_bus();
    for (uint8_t id = AX12_MIN_SERVO_ID; id <= AX12_MAX_SERVO_ID; id++) {
        if (s_dxl.ping(id)) {
            detected_mask |= 1UL << (id - 1U);
            detected_count++;
            ESP_LOGI(TAG, "Servo detectado: ID=%u", id);
        }
    }

    if (detected_count == 0) {
        ESP_LOGW(TAG, "No se recibieron respuestas durante el escaneo");
    } else if (detected_count != AX12_SERVO_COUNT) {
        ESP_LOGW(TAG,
                 "Escaneo incompleto: %u/18 respuestas (mask=0x%05lX)",
                 detected_count,
                 (unsigned long)detected_mask);
    } else {
        ESP_LOGI(TAG, "Escaneo completo: 18/18 respuestas");
    }
    unlock_bus();
}

/* --------------------------------------------------------------------------
 * Configuración de los actuadores
 * -------------------------------------------------------------------------- */

void ax12_configure_eeprom_ram(void)
{
    lock_bus();
    ParamForSyncWriteInst_t sync_packet;
    memset(&sync_packet, 0, sizeof(sync_packet));
    sync_packet.id_count = AX12_SERVO_COUNT;

    for (uint8_t i = 0; i < AX12_SERVO_COUNT; i++) {
        sync_packet.xel[i].id = AX12_MIN_SERVO_ID + i;
    }

    // EEPROM: Return Delay Time.
    sync_packet.addr = AX12_ADDR_RETURN_DELAY;
    sync_packet.length = 1;

    for (uint8_t i = 0; i < AX12_SERVO_COUNT; i++) {
        sync_packet.xel[i].data[0] = AX12_RETURN_DELAY_VALUE;
    }

    if (!s_dxl.syncWrite(sync_packet)) {
        ESP_LOGE(TAG, "No se pudo transmitir Return Delay Time");
    }
    vTaskDelay(pdMS_TO_TICKS(AX12_EEPROM_WRITE_DELAY_MS));

    // EEPROM: Alarm LED y Shutdown.
    sync_packet.addr = AX12_ADDR_ALARM_LED;
    sync_packet.length = AX12_PROTECTION_DATA_SIZE;

    for (uint8_t i = 0; i < AX12_SERVO_COUNT; i++) {
        sync_packet.xel[i].data[0] = AX12_ALARM_LED_MASK;
        sync_packet.xel[i].data[1] = AX12_SHUTDOWN_MASK;
    }

    if (!s_dxl.syncWrite(sync_packet)) {
        ESP_LOGE(TAG, "No se pudo transmitir Alarm LED/Shutdown");
    }
    vTaskDelay(pdMS_TO_TICKS(AX12_EEPROM_WRITE_DELAY_MS));

    // RAM: márgenes y pendientes de compliance.
    sync_packet.addr = AX12_ADDR_CW_MARGIN;
    sync_packet.length = AX12_COMPLIANCE_DATA_SIZE;

    for (uint8_t i = 0; i < AX12_SERVO_COUNT; i++) {
        sync_packet.xel[i].data[0] = AX12_COMPLIANCE_MARGIN;
        sync_packet.xel[i].data[1] = AX12_COMPLIANCE_MARGIN;
        sync_packet.xel[i].data[2] = AX12_COMPLIANCE_SLOPE;
        sync_packet.xel[i].data[3] = AX12_COMPLIANCE_SLOPE;
    }

    if (!s_dxl.syncWrite(sync_packet)) {
        ESP_LOGE(TAG, "No se pudo transmitir la configuración de compliance");
    }

    ESP_LOGI(TAG, "Configuración EEPROM y RAM finalizada");
    unlock_bus();
}

/* --------------------------------------------------------------------------
 * Control de par y movimiento
 * -------------------------------------------------------------------------- */

bool ax12_set_torque(uint8_t id, bool enabled)
{
    if (!is_valid_servo_id(id)) return false;

    lock_bus();
    bool ok = enabled ? s_dxl.torqueOn(id) : s_dxl.torqueOff(id);
    unlock_bus();
    return ok;
}

bool ax12_sync_move_deg_speed(const uint8_t *ids,
                              const float *angles,
                              int count,
                              uint16_t speed)
{
    if (!ids || !angles || count <= 0) {
        ESP_LOGE(TAG, "Parámetros inválidos en Sync Write");
        return false;
    }
    if (count > AX12_SERVO_COUNT) {
        ESP_LOGE(TAG, "Count=%d excede el máximo (%d)", count, AX12_SERVO_COUNT);
        return false;
    }

    // Se valida el lote completo antes de empezar a construir el paquete.
    for (int i = 0; i < count; i++) {
        if (!is_valid_servo_id(ids[i])) {
            ESP_LOGE(TAG, "ID inválido en Sync Write: %u", ids[i]);
            return false;
        }
        if (!isfinite(angles[i])) {
            ESP_LOGE(TAG, "Ángulo no finito en Sync Write para ID=%u", ids[i]);
            return false;
        }
        for (int j = 0; j < i; j++) {
            if (ids[i] == ids[j]) {
                ESP_LOGE(TAG, "ID repetido en Sync Write: %u", ids[i]);
                return false;
            }
        }
    }

    // En modo articulación, velocidad cero significa máxima velocidad
    // sin control.
    if (speed == 0) {
        speed = 1;
    } else if (speed > AX12_MAX_SPEED_RAW) {
        speed = AX12_MAX_SPEED_RAW;
    }

    ParamForSyncWriteInst_t sync_packet;
    memset(&sync_packet, 0, sizeof(sync_packet));
    sync_packet.addr = AX12_ADDR_GOAL_POSITION;
    sync_packet.length = AX12_MOVE_DATA_SIZE;
    sync_packet.id_count = (uint8_t)count;

    for (int i = 0; i < count; i++) {
        float degrees = angles[i];
        if (degrees < AX12_MIN_ANGLE_DEG) {
            degrees = AX12_MIN_ANGLE_DEG;
        } else if (degrees > AX12_MAX_ANGLE_DEG) {
            degrees = AX12_MAX_ANGLE_DEG;
        }

        // Conversión lineal de grados a las unidades del registro Goal Position.
        uint16_t position_raw =
            (uint16_t)((degrees / AX12_MAX_ANGLE_DEG) * AX12_MAX_POSITION_RAW);

        sync_packet.xel[i].id = ids[i];
        sync_packet.xel[i].data[0] = (uint8_t)(position_raw & 0xFFU);
        sync_packet.xel[i].data[1] = (uint8_t)((position_raw >> 8U) & 0xFFU);
        sync_packet.xel[i].data[2] = (uint8_t)(speed & 0xFFU);
        sync_packet.xel[i].data[3] = (uint8_t)((speed >> 8U) & 0xFFU);
    }

    lock_bus();
    bool ok = s_dxl.syncWrite(sync_packet);
    unlock_bus();
    return ok;
}

/* --------------------------------------------------------------------------
 * Telemetría
 * -------------------------------------------------------------------------- */

bool ax12_read_health(uint8_t id,
                      uint32_t timeout_ms,
                      ax12_health_t *out_health)
{
    if (!is_valid_servo_id(id) || !out_health) {
        return false;
    }

    memset(out_health, 0, sizeof(*out_health));

    // Present Load, Voltage y Temperature forman un bloque contiguo de 4 bytes.
    uint8_t data[AX12_HEALTH_DATA_SIZE] = {0};
    lock_bus();
    int32_t received = s_dxl.read(id,
                                  AX12_ADDR_PRESENT_LOAD,
                                  AX12_HEALTH_DATA_SIZE,
                                  data,
                                  sizeof(data),
                                  timeout_ms);
    int32_t lib_error = s_dxl.getLastLibErrCode();
    uint8_t status_error = s_dxl.getLastStatusPacketError();
    unlock_bus();
    if (received != AX12_HEALTH_DATA_SIZE || lib_error != DXL_LIB_OK) return false;

    out_health->load_raw = (uint16_t)(data[0] | ((uint16_t)data[1] << 8U));
    out_health->voltage_raw = data[2];
    out_health->temperature_c = data[3];

    // El byte ERR pertenece al mismo paquete de estado, no al mapa de memoria.
    out_health->status_error_raw = status_error;
    return true;
}

bool ax12_read_register(uint8_t id,
                        uint8_t address,
                        uint8_t size,
                        uint32_t timeout_ms,
                        ax12_register_read_t *out_read)
{
    if (!is_valid_servo_id(id) || !out_read ||
        (size != 1 && size != 2) ||
        (uint16_t)address + size > AX12_CONTROL_TABLE_SIZE) {
        return false;
    }

    memset(out_read, 0, sizeof(*out_read));

    uint8_t data[2] = {0};
    lock_bus();
    int32_t received = s_dxl.read(id,
                                  address,
                                  size,
                                  data,
                                  sizeof(data),
                                  timeout_ms);
    int32_t lib_error = s_dxl.getLastLibErrCode();
    uint8_t status_error = s_dxl.getLastStatusPacketError();
    unlock_bus();
    if (received != size || lib_error != DXL_LIB_OK) return false;

    out_read->value = data[0];
    if (size == 2) {
        out_read->value |= (uint16_t)data[1] << 8U;
    }
    out_read->status_error_raw = status_error;
    return true;
}

bool ax12_write_register(uint8_t id,
                         uint8_t address,
                         uint8_t size,
                         uint16_t value,
                         uint32_t timeout_ms,
                         ax12_register_write_t *out_write)
{
    if (!is_valid_servo_id(id) || !out_write ||
        (size != 1 && size != 2) ||
        (uint16_t)address + size > AX12_CONTROL_TABLE_SIZE ||
        (size == 1 && value > 0xFFU)) {
        return false;
    }

    memset(out_write, 0, sizeof(*out_write));

    uint8_t data[2] = {
        (uint8_t)(value & 0xFFU),
        (uint8_t)((value >> 8U) & 0xFFU),
    };

    lock_bus();
    bool written = s_dxl.write(id, address, data, size, timeout_ms);
    int32_t lib_error = s_dxl.getLastLibErrCode();
    uint8_t status_error = s_dxl.getLastStatusPacketError();
    unlock_bus();
    if (lib_error != DXL_LIB_OK) return false;

    out_write->status_error_raw = status_error;
    return written;
}
