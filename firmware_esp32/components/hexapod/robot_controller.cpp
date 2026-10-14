/**
 * @file robot_controller.cpp
 * @brief Implementación del estado compartido del robot.
 * @ingroup group_robot_controller
 */

#include "robot_controller.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "matrix_math.h"

/* --------------------------------------------------------------------------
 * Inicialización
 * -------------------------------------------------------------------------- */

void robot_controller_init(robot_controller_t *rc)
{
    rc->mutex = xSemaphoreCreateMutexStatic(&rc->mutex_buf);
    memset(&rc->st, 0, sizeof(robot_state_t));

    // Estado seguro y parámetros iniciales de marcha y postura.
    rc->st.mode = GAIT_MODE_STOP;
    rc->st.period_s = 2.0f;
    rc->st.step_h = 40.0f;
    rc->st.transition_time_s = 1.0f;
}

/* --------------------------------------------------------------------------
 * Comandos
 * -------------------------------------------------------------------------- */

void robot_set_mode(robot_controller_t *rc, gait_mode_t mode)
{
    if ((int)mode < (int)GAIT_MODE_STOP || mode > GAIT_MODE_AMBLE) mode = GAIT_MODE_STOP;
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.mode = mode;
    xSemaphoreGive(rc->mutex);
}

void robot_set_velocity(robot_controller_t *rc, float vx, float vy, float vyaw)
{
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(vyaw)) vx = vy = vyaw = 0.0f;

    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.v_x = vx;
    rc->st.v_y = vy;
    rc->st.v_yaw = vyaw;
    xSemaphoreGive(rc->mutex);
}

void robot_set_gait_params(robot_controller_t *rc, float period_s, float step_h_mm)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    if (isfinite(period_s) && period_s > 0.0f) {
        rc->st.period_s = (period_s >= 0.6f) ? period_s : 0.6f;
    }
    if (isfinite(step_h_mm) && step_h_mm > 0.0f) {
        rc->st.step_h = scalar_clamp(step_h_mm, 10.0f, 60.0f);
    }
    xSemaphoreGive(rc->mutex);
}

void robot_set_posture(robot_controller_t *rc, float r, float p, float y, float x, float ty, float z)
{
    if (!isfinite(r) || !isfinite(p) || !isfinite(y) || !isfinite(x) || !isfinite(ty) || !isfinite(z)) return;

    // Límites individuales de postura; la IK valida la combinación completa.
    const float MAX_RP_RAD = 0.35f;  // Aproximadamente 20 grados.
    const float MAX_YAW_RAD = 0.52f; // Aproximadamente 30 grados.
    const float MAX_XY_MM = 40.0f;
    const float MAX_Z_MM = 50.0f;

    xSemaphoreTake(rc->mutex, portMAX_DELAY);

    rc->st.roll = scalar_clamp(r, -MAX_RP_RAD, MAX_RP_RAD);
    rc->st.pitch = scalar_clamp(p, -MAX_RP_RAD, MAX_RP_RAD);
    rc->st.yaw = scalar_clamp(y, -MAX_YAW_RAD, MAX_YAW_RAD);
    rc->st.x = scalar_clamp(x, -MAX_XY_MM, MAX_XY_MM);
    rc->st.y = scalar_clamp(ty, -MAX_XY_MM, MAX_XY_MM);
    rc->st.z = scalar_clamp(z, -MAX_Z_MM, MAX_Z_MM);

    xSemaphoreGive(rc->mutex);
}

void robot_set_smooth_time(robot_controller_t *rc, float time_s)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    if (isfinite(time_s) && time_s > 0.0f) {
        rc->st.transition_time_s = (time_s >= 0.1f) ? time_s : 0.1f;
    }
    xSemaphoreGive(rc->mutex);
}

/* --------------------------------------------------------------------------
 * Lectura del estado
 * -------------------------------------------------------------------------- */

robot_state_t robot_get_state(robot_controller_t *rc)
{
    robot_state_t copy;
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    copy = rc->st;
    xSemaphoreGive(rc->mutex);
    return copy;
}

/* --------------------------------------------------------------------------
 * Publicación interna
 * -------------------------------------------------------------------------- */

void robot_report_odometry(robot_controller_t *rc, float x, float y, float yaw, float roll, float pitch, float body_yaw, float bx, float by, float bz)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.odo_x_mm = x;
    rc->st.odo_y_mm = y;
    rc->st.odo_yaw_rad = yaw;
    rc->st.curr_roll = roll;
    rc->st.curr_pitch = pitch;
    rc->st.curr_yaw = body_yaw;
    rc->st.curr_x = bx;
    rc->st.curr_y = by;
    rc->st.curr_z = bz;
    xSemaphoreGive(rc->mutex);
}

void robot_report_servo_health(robot_controller_t *rc, uint8_t id, uint8_t voltage_raw, uint8_t temperature_c, uint16_t load_raw, uint8_t status_error_raw, bool valid)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.servo_health_id = id;
    rc->st.servo_voltage_raw = voltage_raw;
    rc->st.servo_temperature_c = temperature_c;
    rc->st.servo_load_raw = load_raw;
    rc->st.servo_status_error_raw = status_error_raw;
    rc->st.servo_health_valid = valid;
    rc->st.servo_health_seq++;
    xSemaphoreGive(rc->mutex);
}

/* --------------------------------------------------------------------------
 * Acceso a registros AX-12A
 * -------------------------------------------------------------------------- */

void robot_request_servo_register(robot_controller_t *rc, uint8_t id, uint8_t address, uint8_t size)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.servo_register_request_id = id;
    rc->st.servo_register_request_address = address;
    rc->st.servo_register_request_size = size;
    rc->st.servo_register_request_write = false;
    rc->st.servo_register_request_value = 0;
    rc->st.servo_register_request_seq++;
    xSemaphoreGive(rc->mutex);
}

void robot_request_servo_register_write(robot_controller_t *rc, uint8_t id, uint8_t address, uint8_t size, uint16_t value)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.servo_register_request_id = id;
    rc->st.servo_register_request_address = address;
    rc->st.servo_register_request_size = size;
    rc->st.servo_register_request_write = true;
    rc->st.servo_register_request_value = value;
    rc->st.servo_register_request_seq++;
    xSemaphoreGive(rc->mutex);
}

void robot_report_servo_register(robot_controller_t *rc, uint8_t id, uint8_t address, uint8_t size, uint16_t value, uint8_t status_error_raw, bool valid)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.servo_register_result_id = id;
    rc->st.servo_register_result_address = address;
    rc->st.servo_register_result_size = size;
    rc->st.servo_register_result_value = value;
    rc->st.servo_register_result_error = status_error_raw;
    rc->st.servo_register_result_valid = valid;
    rc->st.servo_register_result_write = false;
    rc->st.servo_register_result_seq++;
    xSemaphoreGive(rc->mutex);
}

void robot_report_servo_register_write(robot_controller_t *rc, uint8_t id, uint8_t address, uint8_t size, uint16_t value, uint8_t status_error_raw, bool valid)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.servo_register_result_id = id;
    rc->st.servo_register_result_address = address;
    rc->st.servo_register_result_size = size;
    rc->st.servo_register_result_value = value;
    rc->st.servo_register_result_error = status_error_raw;
    rc->st.servo_register_result_valid = valid;
    rc->st.servo_register_result_write = true;
    rc->st.servo_register_result_seq++;
    xSemaphoreGive(rc->mutex);
}

/* --------------------------------------------------------------------------
 * Odometría
 * -------------------------------------------------------------------------- */

void robot_reset_odometry(robot_controller_t *rc)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    rc->st.reset_odometry_req = true;
    xSemaphoreGive(rc->mutex);
}

bool robot_consume_odometry_reset(robot_controller_t *rc)
{
    xSemaphoreTake(rc->mutex, portMAX_DELAY);
    bool requested = rc->st.reset_odometry_req;
    rc->st.reset_odometry_req = false;
    xSemaphoreGive(rc->mutex);
    return requested;
}
