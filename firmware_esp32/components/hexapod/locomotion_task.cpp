/**
 * @file locomotion_task.cpp
 * @brief Implementación de la tarea principal de locomoción.
 * @ingroup group_locomotion_task
 */

#include "locomotion_task.h"
#include <string.h>
#include "freertos/task.h"
#include "esp_log.h"
#include "ax12.h"
#include "diagnostic.h"
#include "gait_engine.h"
#include "body_kinematics.h"
#include "leg_kinematics.h"
#include "posture_engine.h"

static const char *TAG = "LOCOMOTION"; /**< Etiqueta de los mensajes de registro. */

/* Posiciones neutrales de las patas en BODY, en milímetros. */
static const vec3 NEUTRAL_BODY[LEG_COUNT] = {
    {+180.0f, +170.0f, -100.0f},
    {   0.0f, +240.0f, -100.0f},
    {-180.0f, +170.0f, -100.0f},
    {-180.0f, -170.0f, -100.0f},
    {   0.0f, -240.0f, -100.0f},
    {+180.0f, -170.0f, -100.0f},
};

/* Estado persistente de la tarea y última solución conjunta válida. */
typedef struct {
    robot_controller_t *rc;              /**< Estado compartido con las interfaces externas. */
    gait_engine_t gait;                  /**< Generador de fases y trayectorias de las patas. */
    body_kinematics_t bk;                /**< Postura, trayectoria y odometría del cuerpo. */
    posture_engine_t posture;            /**< Suavizado de la postura local del chasis. */
    float gait_change_remaining_s;       /**< Tiempo restante de neutralización, en s. */
    bool ik_fault;                       /**< true mientras el último tick tenga una IK inválida. */
    float last_deg[LEG_COUNT][3];        /**< Última consigna conjunta válida, en grados. */
    uint8_t servo_ids[LEG_COUNT * 3];    /**< IDs asociados a las consignas anteriores. */
} locomotion_ctx_t;

/* --------------------------------------------------------------------------
 * Cambio del modo de marcha
 * -------------------------------------------------------------------------- */

/* Actualiza la marcha y mantiene dos periodos a objetivo cero antes de cambiar
 * entre patrones activos. STOP conserva el último patrón interno. */
static void update_gait(locomotion_ctx_t *ctx, const robot_state_t *st, float dt_s)
{
    gait_mode_t requested_mode = st->mode;
    bool changing = requested_mode != GAIT_MODE_STOP && requested_mode != ctx->gait.mode;

    if (changing && ctx->gait_change_remaining_s <= 0.0f) ctx->gait_change_remaining_s = 2.0f * ctx->gait.period_s;
    if (!changing) ctx->gait_change_remaining_s = 0.0f;

    bool params_changed = fabsf(st->period_s - ctx->gait.period_s) > 0.001f || fabsf(st->step_h - ctx->gait.step_h) > 0.001f;
    if (!changing && requested_mode != GAIT_MODE_STOP && params_changed) gait_engine_configure(&ctx->gait, requested_mode, st->period_s, st->step_h);

    float command_period = requested_mode != GAIT_MODE_STOP && !changing ? ctx->gait.period_s : 0.0f;
    gait_engine_set_command(&ctx->gait, st->v_x * command_period, st->v_y * command_period, st->v_yaw * command_period);
    gait_engine_step(&ctx->gait, &ctx->bk, dt_s);

    if (changing && (ctx->gait_change_remaining_s -= dt_s) <= 0.0f) {
        gait_engine_configure(&ctx->gait, requested_mode, st->period_s, st->step_h);
        ctx->gait_change_remaining_s = 0.0f;
    }
}

/* --------------------------------------------------------------------------
 * Pipeline cinemático
 * -------------------------------------------------------------------------- */

/* Calcula el ciclo sobre una copia provisional y solo lo confirma cuando las
 * seis patas producen una IK válida. */
static bool process_robot_physics(locomotion_ctx_t *ctx, float dt_s)
{
    /* 1. Tomar una instantánea del comando y trabajar sobre una copia.
     * Ningún cálculo cinemático modifica todavía el estado persistente. */
    robot_state_t st = robot_get_state(ctx->rc);
    locomotion_ctx_t candidate = *ctx;

    /* 2. Avanzar provisionalmente la postura suavizada y construir T_body. */
    posture_engine_set_transition_time(&candidate.posture, st.transition_time_s);
    posture_engine_set_target(&candidate.posture, st.roll, st.pitch, st.yaw, st.x, st.y, st.z);
    posture_engine_step(&candidate.posture, dt_s);
    body_kinematics_set_posture_rpy_xyz(&candidate.bk,candidate.posture.curr_roll, candidate.posture.curr_pitch, candidate.posture.curr_yaw,
                                        candidate.posture.curr_x, candidate.posture.curr_y, candidate.posture.curr_z);

    /* 3. Avanzar provisionalmente la marcha, las patas y la odometría. */
    update_gait(&candidate, &st, dt_s);

    /* 4. Resolver las seis IK. La máscara conserva todas las patas inválidas
     * para informar del fallo sin abandonar el bucle en la primera. */
    uint32_t invalid_legs = 0;
    float candidate_deg[LEG_COUNT][3];
    for (int i = 0; i < LEG_COUNT; i++) {
        if (!leg_compute_deg(leg_get_cfg((leg_id_t)i),
                             candidate.gait.foot_body[i].x,
                             candidate.gait.foot_body[i].y,
                             candidate.gait.foot_body[i].z,
                             candidate_deg[i])) {
            invalid_legs |= (1U << i);
        }
    }

    bool ik_ok = invalid_legs == 0;
    if (ik_ok) {
        /* 5. Confirmar atómicamente el ciclo provisional completo. */
        bool mode_changed = candidate.gait.mode != ctx->gait.mode;
        bool ik_recovered = ctx->ik_fault;

        memcpy(candidate.last_deg, candidate_deg, sizeof(candidate_deg));
        candidate.ik_fault = false;
        *ctx = candidate;

        if (mode_changed) ESP_LOGI(TAG, "Gait Changed: Mode %d, Duty %.2f", st.mode, ctx->gait.duty);
        if (ik_recovered) ESP_LOGI(TAG, "IK recuperada; locomoción reanudada");
    } else {
        /* Conservar el último ciclo válido y registrar una sola vez el fallo. */
        if (!ctx->ik_fault) ESP_LOGE(TAG, "IK inválida en patas 0x%02lX; tick descartado", (unsigned long)invalid_legs);
        ctx->ik_fault = true;

        /* Anular el desplazamiento confirmado. La rampa del comando solo avanza
         * en el candidato descartado; sin esto, un pie en apoyo fuera de alcance
         * bloquearía la marcha incluso con velocidad nula. Con CMD,0,0,0 el
         * siguiente tick no mueve el cuerpo, la IK vuelve a ser válida y la fase
         * reanuda su avance. */
        gait_engine_halt(&ctx->gait);
    }

    /* 6. El reset solo redefine el origen global. Sincronizar bk_prev evita
     * interpretar el cambio de coordenadas como movimiento real del cuerpo. */
    if (robot_consume_odometry_reset(ctx->rc)) {
        body_kinematics_reset_odometry(&ctx->bk);
        ctx->gait.bk_prev = ctx->bk;
        ESP_LOGI(TAG, "Odometría reiniciada");
    }

    /* 7. Publicar siempre el último estado confirmado para ROS y telemetría. */
    float odo_x_mm, odo_y_mm, odo_yaw_rad;
    body_kinematics_get_odometry(&ctx->bk, &odo_x_mm, &odo_y_mm, &odo_yaw_rad);
    robot_report_odometry(ctx->rc,
                          odo_x_mm, odo_y_mm, odo_yaw_rad,
                          ctx->posture.curr_roll, ctx->posture.curr_pitch, ctx->posture.curr_yaw,
                          ctx->posture.curr_x, ctx->posture.curr_y, ctx->posture.curr_z);

    /* Solo una solución conjunta válida autoriza el Sync Write de este tick. */
    return ik_ok;
}

/* --------------------------------------------------------------------------
 * Tarea FreeRTOS
 * -------------------------------------------------------------------------- */

/* Inicializa el estado y ejecuta el bucle periódico de 20 ms. */
static void locomotion_task_entry(void *arg)
{
    locomotion_ctx_t *ctx = (locomotion_ctx_t*)arg;

    int servo_index = 0;
    for (int i = 0; i < LEG_COUNT; i++) {
        const leg_cfg_t *leg_cfg = leg_get_cfg((leg_id_t)i);
        ctx->servo_ids[servo_index++] = leg_cfg->id_coxa;
        ctx->servo_ids[servo_index++] = leg_cfg->id_femur;
        ctx->servo_ids[servo_index++] = leg_cfg->id_tibia;
    }

    /* Inicializar UART2 desde esta tarea fija al núcleo 1. De este modo, la
     * interrupción del driver y las transmisiones DYNAMIXEL comparten núcleo. */
    ax12_init(1000000);
    ax12_scan();
    ax12_configure_eeprom_ram();
    for (int i = 0; i < LEG_COUNT * 3; i++) ax12_set_torque(ctx->servo_ids[i], true);

    /* Inicialización de los motores matemáticos. */
    body_kinematics_init(&ctx->bk, 100.0f); //Al iniciar, el chasis se encuentra a 100 mm sobre el suelo
    robot_state_t st = robot_get_state(ctx->rc);
    posture_engine_init(&ctx->posture, st.transition_time_s);

    /* El patrón interno inicial es Trípode incluso si el estado solicita STOP. */
    gait_engine_init(&ctx->gait, GAIT_MODE_TRIPOD, st.period_s, st.step_h, NEUTRAL_BODY);

    ctx->gait_change_remaining_s = 0.0f;
    ctx->ik_fault = false;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t loop_period = pdMS_TO_TICKS(20);

    while (1) {
        /* Cálculo y transmisión conjunta de las consignas. */
        if (process_robot_physics(ctx, (float)loop_period / (float)configTICK_RATE_HZ)) ax12_sync_move_deg_speed(ctx->servo_ids, &ctx->last_deg[0][0], LEG_COUNT * 3, 500);

        diagnostic_notify_cycle();

        /* Espera absoluta hasta el siguiente instante de activación. */
        vTaskDelayUntil(&xLastWakeTime, loop_period);
    }
}

void locomotion_task_start(robot_controller_t *rc)
{
    static locomotion_ctx_t ctx;
    ctx.rc = rc;

    xTaskCreatePinnedToCore(locomotion_task_entry, "locomotion", 8192, &ctx, 10, NULL, 1);
}
