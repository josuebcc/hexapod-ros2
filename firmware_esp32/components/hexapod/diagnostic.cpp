/**
 * @file diagnostic.cpp
 * @brief Implementación de la supervisión auxiliar de los servos AX-12A.
 * @ingroup group_diagnostic
 */

#include "diagnostic.h"

#include <string.h>
#include "ax12.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "leg_config.h"

/* --------------------------------------------------------------------------
 * Configuración y estado privado
 * -------------------------------------------------------------------------- */

static const char *TAG = "DIAGNOSTIC";                    /**< Etiqueta de los mensajes de registro. */
static const uint32_t AX12_HEALTH_TIMEOUT_MS = 8;         /**< Timeout de telemetría, en ms. */
static const uint32_t AX12_REGISTER_TIMEOUT_MS = 8;       /**< Timeout de acceso a registros, en ms. */

/**
 * @brief Contexto privado de la tarea de diagnóstico.
 *
 * Conserva la referencia al estado compartido, el orden circular de consulta
 * de los actuadores y las secuencias necesarias para detectar peticiones
 * nuevas. La instancia tiene almacenamiento estático.
 */
typedef struct {
    robot_controller_t *rc;                    /**< Estado compartido del robot. */
    uint8_t servo_ids[LEG_COUNT * 3];          /**< IDs en el orden definido por @ref g_leg_cfgs. */
    uint8_t next_health_index;                 /**< Índice del próximo servo que se consultará. */
    uint32_t last_register_request_seq;        /**< Última petición de registro atendida. */
    TaskHandle_t task_handle;                  /**< Handle de la tarea FreeRTOS. */
} diagnostic_ctx_t;

static diagnostic_ctx_t s_diagnostic; /**< Única instancia estática del diagnóstico. */

/* --------------------------------------------------------------------------
 * Telemetría periódica
 * -------------------------------------------------------------------------- */

/**
 * @brief Consulta y publica la telemetría del siguiente servo.
 *
 * Lee tensión, temperatura, carga y el byte de error. La muestra se publica
 * aunque la transacción falle, utilizando el campo `valid` para distinguir el
 * resultado. Después avanza circularmente al siguiente ID de la lista.
 *
 * @param[in,out] ctx Contexto de diagnóstico.
 */
static void poll_next_servo_health(diagnostic_ctx_t *ctx)
{
    const uint8_t id = ctx->servo_ids[ctx->next_health_index];
    ax12_health_t health = {};
    bool ok = ax12_read_health(id, AX12_HEALTH_TIMEOUT_MS, &health);
    robot_report_servo_health(ctx->rc, id, health.voltage_raw, health.temperature_c, health.load_raw, health.status_error_raw, ok);

    ctx->next_health_index++;
    if (ctx->next_health_index >= LEG_COUNT * 3) ctx->next_health_index = 0;
}

/* --------------------------------------------------------------------------
 * Acceso a registros bajo demanda
 * -------------------------------------------------------------------------- */

/**
 * @brief Atiende una petición pendiente de lectura o escritura de registro.
 *
 * La secuencia almacenada en @ref robot_state_t permite detectar una petición
 * nueva. Primero se copia el estado compartido y se actualiza la última
 * secuencia atendida; la transacción AX-12A se ejecuta después sin mantener el
 * mutex del controlador.
 *
 * Una petición de escritura publica el valor solicitado y el estado de la
 * transacción, pero no realiza una lectura posterior de verificación.
 *
 * @param[in,out] ctx Contexto de diagnóstico.
 * @retval true Se procesó una petición, aunque la transacción pudiera fallar.
 * @retval false No había ninguna petición nueva.
 */
static bool process_servo_register_request(diagnostic_ctx_t *ctx)
{
    robot_state_t robot_state = robot_get_state(ctx->rc);
    if (robot_state.servo_register_request_seq == ctx->last_register_request_seq) return false;

    ctx->last_register_request_seq = robot_state.servo_register_request_seq;

    if (robot_state.servo_register_request_write) {
        ax12_register_write_t result = {};
        bool ok = ax12_write_register(robot_state.servo_register_request_id,
                                      robot_state.servo_register_request_address,
                                      robot_state.servo_register_request_size,
                                      robot_state.servo_register_request_value,
                                      AX12_REGISTER_TIMEOUT_MS,
                                      &result);
        robot_report_servo_register_write(ctx->rc,
                                          robot_state.servo_register_request_id,
                                          robot_state.servo_register_request_address,
                                          robot_state.servo_register_request_size,
                                          robot_state.servo_register_request_value,
                                          result.status_error_raw,
                                          ok);
    } else {
        ax12_register_read_t result = {};
        bool ok = ax12_read_register(robot_state.servo_register_request_id,
                                     robot_state.servo_register_request_address,
                                     robot_state.servo_register_request_size,
                                     AX12_REGISTER_TIMEOUT_MS,
                                     &result);
        robot_report_servo_register(ctx->rc,
                                    robot_state.servo_register_request_id,
                                    robot_state.servo_register_request_address,
                                    robot_state.servo_register_request_size,
                                    result.value,
                                    result.status_error_raw,
                                    ok);
    }

    return true;
}

/**
 * @brief Punto de entrada de la tarea FreeRTOS de diagnóstico.
 *
 * En cada activación atiende prioritariamente una petición REG/WREG. Si no
 * existe ninguna, consulta la telemetría del siguiente servo de la lista.
 * De este modo realiza como máximo una transacción auxiliar por ciclo.
 *
 * `ulTaskNotifyTake()` mantiene la tarea bloqueada sin consumir CPU. El
 * parámetro `pdTRUE` vacía conjuntamente las notificaciones acumuladas, por lo
 * que diagnóstico no intenta recuperar consultas omitidas.
 *
 * @param[in] arg Puntero a @ref diagnostic_ctx_t.
 */
static void diagnostic_task_entry(void *arg)
{
    diagnostic_ctx_t *ctx = (diagnostic_ctx_t*)arg;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!process_servo_register_request(ctx)) poll_next_servo_health(ctx);
    }
}

/* --------------------------------------------------------------------------
 * API pública
 * -------------------------------------------------------------------------- */

bool diagnostic_start(robot_controller_t *rc)
{
    if (!rc) return false;

    memset(&s_diagnostic, 0, sizeof(s_diagnostic));
    s_diagnostic.rc = rc;

    int servo_index = 0;
    for (int i = 0; i < LEG_COUNT; i++) {
        s_diagnostic.servo_ids[servo_index++] = g_leg_cfgs[i].id_coxa;
        s_diagnostic.servo_ids[servo_index++] = g_leg_cfgs[i].id_femur;
        s_diagnostic.servo_ids[servo_index++] = g_leg_cfgs[i].id_tibia;
    }

    if (xTaskCreatePinnedToCore(diagnostic_task_entry,
                                "diagnostic",
                                4096,
                                &s_diagnostic,
                                9,
                                &s_diagnostic.task_handle,
                                1) == pdPASS) return true;

    s_diagnostic.task_handle = NULL;
    ESP_LOGE(TAG, "No se pudo crear la tarea de diagnóstico");
    return false;
}

void diagnostic_notify_cycle(void)
{
    if (s_diagnostic.task_handle) xTaskNotifyGive(s_diagnostic.task_handle);
}
