/**
 * @file serial_bridge.cpp
 * @brief Implementación del puente Serial (UART1) para integración con ROS 2.
 * @ingroup group_serial_bridge
 */

#include "serial_bridge.h"

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* --------------------------------------------------------------------------
 * Configuración privada
 * -------------------------------------------------------------------------- */

static const char *TAG = "SERIAL_BRIDGE"; /**< Etiqueta de los mensajes de registro. */

#define UART_BRIDGE_NUM UART_NUM_1 /**< Puerto UART empleado por el puente. */
#define UART_TX_PIN GPIO_NUM_4     /**< Pin TX, conectado al RX de la Raspberry Pi. */
#define UART_RX_PIN GPIO_NUM_5     /**< Pin RX, conectado al TX de la Raspberry Pi. */
#define BUF_SIZE 256               /**< Capacidad de los buffers de recepción. */

/* --------------------------------------------------------------------------
 * Ayuda del protocolo
 * -------------------------------------------------------------------------- */

/**
 * @brief Publica la ayuda del protocolo ASCII.
 */
static void bridge_write_help(void)
{
    static const char *help_text =
        "HELP,Comandos disponibles:\n"
        "HELP o ?                  -> muestra esta ayuda\n"
        "MOD,0|1|2|3               -> modo 0=Stop 1=Tripod 2=Ripple 3=Amble\n"
        "CMD,vx,vy,vyaw            -> velocidad lineal y giro\n"
        "POS,roll,pitch,yaw,x,y,z  -> postura del cuerpo\n"
        "GAT,period_s,step_h_mm    -> parametros de marcha\n"
        "SMO,time_s                -> tiempo de suavizado\n"
        "REG,id,address,size       -> lee un registro AX-12A de 1 o 2 bytes\n"
        "WREG,id,address,size,value -> escribe 1 o 2 bytes de un registro AX-12A\n"
        "RST                       -> resetea odometria\n"
        "ODO,x,y,yaw,roll,pitch,body_yaw,x_body,y_body,z_body -> respuesta de odometria y postura\n";

    uart_write_bytes(UART_BRIDGE_NUM, help_text, (int)strlen(help_text));
}

/* --------------------------------------------------------------------------
 * Procesamiento de comandos
 * -------------------------------------------------------------------------- */

/**
 * @brief Interpreta y ejecuta una línea completa del protocolo.
 *
 * Los comandos de control se delegan en @ref group_robot_controller. Las
 * peticiones de registros se validan aquí antes de publicarlas y serán
 * atendidas posteriormente por @ref group_diagnostic. Las líneas que no
 * coinciden con ningún formato se descartan sin modificar el estado.
 *
 * @param[in,out] rc Controlador compartido.
 * @param[in] line_buffer Comando terminado en `\0` y sin salto de línea.
 */
static void handle_bridge_line(robot_controller_t *rc, const char *line_buffer)
{
    float f1, f2, f3, f4, f5, f6;
    int i1, i2, i3, i4;

    // HELP: mostrar la lista de mensajes admitidos.
    if (strcmp(line_buffer, "HELP") == 0 || strcmp(line_buffer, "?") == 0) bridge_write_help();

    // CMD,vx,vy,vyaw: actualizar la velocidad lineal y angular.
    else if (sscanf(line_buffer, "CMD,%f,%f,%f", &f1, &f2, &f3) == 3) robot_set_velocity(rc, f1, f2, f3);

    // MOD,mode: seleccionar STOP, TRIPOD, RIPPLE o AMBLE.
    else if (sscanf(line_buffer, "MOD,%d", &i1) == 1) robot_set_mode(rc, (gait_mode_t)i1);

    // POS,roll,pitch,yaw,x,y,z: actualizar la postura objetivo del chasis.
    else if (sscanf(line_buffer, "POS,%f,%f,%f,%f,%f,%f", &f1, &f2, &f3, &f4, &f5, &f6) == 6) robot_set_posture(rc, f1, f2, f3, f4, f5, f6);

    // GAT,period,step_h: actualizar el periodo y la altura del paso.
    else if (sscanf(line_buffer, "GAT,%f,%f", &f1, &f2) == 2) robot_set_gait_params(rc, f1, f2);

    // SMO,time: actualizar el tiempo de transición de la postura.
    else if (sscanf(line_buffer, "SMO,%f", &f1) == 1) robot_set_smooth_time(rc, f1);

    // REG,id,address,size: solicitar la lectura de uno o dos bytes.
    else if (sscanf(line_buffer, "REG,%d,%d,%d", &i1, &i2, &i3) == 3) {
        bool valid_request = i1 >= 1 && i1 <= 18 && i2 >= 0 && i2 < 50 && (i3 == 1 || i3 == 2) && i2 + i3 <= 50;
        if (valid_request) robot_request_servo_register(rc, (uint8_t)i1, (uint8_t)i2, (uint8_t)i3);
        else ESP_LOGW(TAG, "Petición REG inválida: %s", line_buffer);
    }

    // WREG,id,address,size,value: solicitar la escritura de uno o dos bytes.
    else if (sscanf(line_buffer, "WREG,%d,%d,%d,%d", &i1, &i2, &i3, &i4) == 4) {
        bool valid_request = i1 >= 1 && i1 <= 18 && i2 >= 0 && i2 < 50 && (i3 == 1 || i3 == 2) && i2 + i3 <= 50 && i4 >= 0 && i4 <= 0xFFFF && (i3 == 2 || i4 <= 0xFF);
        if (valid_request) robot_request_servo_register_write(rc, (uint8_t)i1, (uint8_t)i2, (uint8_t)i3, (uint16_t)i4);
        else ESP_LOGW(TAG, "Petición WREG inválida: %s", line_buffer);
    }

    // RST: solicitar que locomotion_task redefina el origen odométrico.
    else if (strncmp(line_buffer, "RST", 3) == 0) {
        robot_reset_odometry(rc);
        ESP_LOGI(TAG, "Odometría reseteada mediante el puerto serie");
    }
}

/* --------------------------------------------------------------------------
 * Tarea FreeRTOS
 * -------------------------------------------------------------------------- */

/**
 * @brief Punto de entrada de la tarea del puente serie.
 *
 * Recibe comandos delimitados por salto de línea, publica periódicamente la
 * odometría y reenvía las muestras nuevas de diagnóstico por UART1.
 *
 * @param[in] arg Puntero al @ref robot_controller_t compartido.
 */
static void serial_bridge_task(void *arg)
{
    // Recuperar el controlador compartido recibido al crear la tarea.
    robot_controller_t *rc = (robot_controller_t*)arg;

    // `data` recibe bloques de UART; `line_buffer` reconstruye un comando completo.
    uint8_t data[BUF_SIZE];
    char line_buffer[BUF_SIZE];
    int line_idx = 0;

    // Configurar UART1 a 115200 bit/s, 8N1 y sin control de flujo.
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    uart_param_config(UART_BRIDGE_NUM, &uart_config);
    uart_set_pin(UART_BRIDGE_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_BRIDGE_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "Puente serie iniciado en UART1 (TX:%d, RX:%d) a 115200 bit/s", UART_TX_PIN, UART_RX_PIN);

    // Referencias necesarias para temporizar ODO y no repetir resultados ya enviados.
    TickType_t last_odom_time = xTaskGetTickCount();
    uint32_t last_servo_health_seq = 0;
    uint32_t last_servo_register_seq = 0;

    while (1) {
        // Esperar como máximo 10 ms para poder atender también las transmisiones periódicas.
        int len = uart_read_bytes(UART_BRIDGE_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(10));

        // Reconstruir los comandos, que pueden llegar repartidos entre varias lecturas.
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];

                // El salto de línea cierra el string con '\0', lo procesa y reinicia el buffer.
                if (c == '\n') {
                    line_buffer[line_idx] = '\0';
                    handle_bridge_line(rc, line_buffer);
                    line_idx = 0;
                }
                // Ignorar '\r' y reservar siempre una posición para el terminador '\0'.
                else if (c != '\r' && line_idx < BUF_SIZE - 1) line_buffer[line_idx++] = c;
            }
        }

        // Usar la misma instantánea atómica del controlador para todas las salidas.
        robot_state_t robot_state = robot_get_state(rc);

        // Publicar automáticamente la odometría y la postura cada segundo.
        if (xTaskGetTickCount() - last_odom_time >= pdMS_TO_TICKS(1000)) {
            char out_str[128];
            int out_len = snprintf(out_str, sizeof(out_str), "ODO,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f\n",
                                   robot_state.odo_x_mm, robot_state.odo_y_mm, robot_state.odo_yaw_rad, robot_state.curr_roll, robot_state.curr_pitch,
                                   robot_state.curr_yaw, robot_state.curr_x, robot_state.curr_y, robot_state.curr_z);
            uart_write_bytes(UART_BRIDGE_NUM, out_str, out_len);
            last_odom_time = xTaskGetTickCount();
        }

        // Reenviar cada muestra AXH una sola vez, detectándola mediante su secuencia.
        if (robot_state.servo_health_seq != last_servo_health_seq) {
            char out_str[128];
            int out_len = snprintf(out_str, sizeof(out_str), "AXH,%u,%u,%u,%u,%u,%u\n",
                                   robot_state.servo_health_id, robot_state.servo_health_valid ? 1 : 0, robot_state.servo_voltage_raw,
                                   robot_state.servo_temperature_c, robot_state.servo_load_raw, robot_state.servo_status_error_raw);
            uart_write_bytes(UART_BRIDGE_NUM, out_str, out_len);
            last_servo_health_seq = robot_state.servo_health_seq;
        }

        // Reenviar cada resultado AXR/AXW una sola vez, detectándolo mediante su secuencia.
        if (robot_state.servo_register_result_seq != last_servo_register_seq) {
            char out_str[96];
            int out_len = snprintf(out_str, sizeof(out_str), "%s,%u,%u,%u,%u,%u,%u\n",
                                   robot_state.servo_register_result_write ? "AXW" : "AXR", robot_state.servo_register_result_id,
                                   robot_state.servo_register_result_address, robot_state.servo_register_result_size,
                                   robot_state.servo_register_result_valid ? 1 : 0, robot_state.servo_register_result_value,
                                   robot_state.servo_register_result_error);
            uart_write_bytes(UART_BRIDGE_NUM, out_str, out_len);
            last_servo_register_seq = robot_state.servo_register_result_seq;
        }
    }
}

/* --------------------------------------------------------------------------
 * API pública
 * -------------------------------------------------------------------------- */

void serial_bridge_start(robot_controller_t *rc)
{
    // Prioridad 4 en el núcleo 0: el puente queda separado del control cinemático.
    xTaskCreatePinnedToCore(serial_bridge_task, "serial_bridge", 4096, rc, 4, NULL, 0);
}
