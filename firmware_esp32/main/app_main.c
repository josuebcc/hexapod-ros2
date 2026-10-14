/**
 * @file app_main.c
 * @brief Inicialización y arranque del firmware del hexápodo.
 */

#include "esp_log.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "diagnostic.h"
#include "robot_controller.h"
#include "locomotion_task.h"
#include "serial_bridge.h"

static const char *TAG = "MAIN"; /**< Etiqueta de los mensajes de registro. */

/* --------------------------------------------------------------------------
 * Puesta en marcha
 * -------------------------------------------------------------------------- */

/**
 * @brief Inicializa el hardware y arranca las tareas del hexápodo.
 *
 * Crea primero el estado compartido y arranca las tareas de diagnóstico,
 * locomoción y comunicación serie. La propia tarea de locomoción inicializa
 * el bus AX-12A en su núcleo antes de comenzar el bucle periódico. El
 * controlador tiene almacenamiento estático porque las tres tareas conservan
 * un puntero a él.
 *
 * @pre La alimentación de los actuadores y las conexiones UART deben estar
 *      disponibles antes de iniciar el firmware.
 */
static void start_hexapod(void)
{
    ESP_LOGI(TAG, "Iniciando sistema del hexápodo por UART1...");

    // Crear el estado compartido antes de arrancar cualquiera de sus consumidores.
    static robot_controller_t robot;
    robot_controller_init(&robot);

    // Arrancar las tres tareas que comparten el controlador del robot.
    diagnostic_start(&robot);
    locomotion_task_start(&robot);
    serial_bridge_start(&robot);

    ESP_LOGI(TAG, "Sistema listo. Esperando comandos de la Raspberry Pi por UART1.");
}

/**
 * @brief Punto de entrada de la aplicación ESP-IDF.
 *
 * Desactiva el detector de brownout del SoC y arranca el sistema completo.
 */
void app_main(void)
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    start_hexapod();
}
