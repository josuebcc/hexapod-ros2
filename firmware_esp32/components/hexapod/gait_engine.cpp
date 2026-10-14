/**
 * @file gait_engine.cpp
 * @brief Implementación del generador de fases y trayectorias de marcha.
 * @ingroup group_gait_engine
 */

#include "gait_engine.h"

#include <math.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Patrones de marcha
 * -------------------------------------------------------------------------- */

/**
 * @brief Desfases de fase local por modo y pata.
 *
 * La tabla se indexa como
 * `[mode - GAIT_MODE_TRIPOD][leg]`. Las columnas conservan el orden de
 * @ref leg_id_t: LT, LM, LB, RB, RM y RT. La fase local se obtiene mediante
 * `phase_leg = wrap01(phase_global + offset)`, por lo que cada valor adelanta
 * el oscilador de la pata correspondiente. Para que la pata aterrice con un
 * retraso phi respecto a LT, como en los diagramas de marcha de la memoria,
 * el desfase almacenado es `1 - phi`.
 */
static const float GAIT_PHASE_OFFSETS[3][LEG_COUNT] = {
    // Trípode, desde fase global cero: {LM, RB, RT} -> {LT, LB, RM}.
    { 0.0f, 1.0f / 2.0f, 0.0f, 1.0f / 2.0f, 0.0f, 1.0f / 2.0f },

    // Ripple, desde fase global cero: RM -> LB -> RT -> LM -> RB -> LT.
    { 0.0f, 2.0f / 6.0f, 4.0f / 6.0f, 1.0f / 6.0f, 5.0f / 6.0f, 3.0f / 6.0f },

    // Amble, desde fase global cero: {LM, RM} -> {LT, RB} -> {LB, RT}.
    { 1.0f / 3.0f, 2.0f / 3.0f, 0.0f, 1.0f / 3.0f, 2.0f / 3.0f, 0.0f }
};

/**
 * @brief Factor de servicio asociado a cada modo de marcha.
 *
 * El orden de las entradas es Trípode, Ripple y Amble; STOP no representa un
 * patrón interno del generador.
 */
static const float GAIT_DUTY[3] = {
    1.0f / 2.0f, // GAIT_MODE_TRIPOD
    5.0f / 6.0f, // GAIT_MODE_RIPPLE
    2.0f / 3.0f  // GAIT_MODE_AMBLE
};

/**
 * @brief Límites del paso efectivo utilizados como protección cinemática.
 *
 * La restricción combinada actúa como protección interna y conserva la
 * dirección del comando al escalar simultáneamente sus tres componentes.
 */
static const float GAIT_MAX_STEP_X_MM = 120.0f;      /**< Límite longitudinal, en mm. */
static const float GAIT_MAX_STEP_Y_MM = 50.0f;       /**< Límite lateral, en mm. */
static const float GAIT_MAX_STEP_YAW_RAD = 0.35f;    /**< Límite angular, en rad. */
static const float GAIT_MAX_STEP_HEIGHT_MM = 60.0f;  /**< Altura máxima, en mm. */

/**
 * @brief Ciclos de marcha necesarios para pasar de paso nulo a paso máximo.
 *
 * El destino de cada swing se fija al despegar con el paso vigente. Si el paso
 * cambia mucho durante el ciclo siguiente, el pie termina el apoyo lejos de su
 * recorrido nominal y el centro de masas puede salir del polígono de soporte.
 * La rampa acota ese cambio al acelerar y al invertir el sentido.
 */
static const float GAIT_RAMP_CYCLES = 1.0f;

/* --------------------------------------------------------------------------
 * Funciones auxiliares
 * -------------------------------------------------------------------------- */

/**
 * @brief Envuelve una fase en el intervalo [0, 1).
 *
 * @param[in] x Fase de entrada, sin restricción de intervalo.
 * @return Fase equivalente en [0, 1); por ejemplo, 1.2 produce 0.2.
 */
static inline float wrap01(float x)
{
    x = fmodf(x, 1.0f);
    if (x < 0.0f) x += 1.0f;
    return x;
}

/**
 * @brief Acerca una componente del paso a su objetivo.
 *
 * Al reducir el paso sin cambiar de sentido aplica un filtro de primer orden,
 * de modo que la parada mantiene su respuesta rápida. Al acelerar o al invertir
 * el sentido limita la variación a una rampa que alcanza el paso máximo en
 * @ref GAIT_RAMP_CYCLES ciclos.
 *
 * @param[in] current Paso aplicado en el tick anterior.
 * @param[in] target Paso objetivo.
 * @param[in] max_step Recorrido máximo por fase de apoyo de esta componente.
 * @param[in] duty Factor de servicio de la marcha activa.
 * @param[in] k Fracción del ciclo transcurrida en este tick.
 * @param[in] alpha Coeficiente del filtro de frenada.
 * @return Paso que se aplicará en este tick.
 */
static inline float ramp_stride(float current, float target, float max_step, float duty, float k, float alpha)
{
    bool braking = target * current >= 0.0f && fabsf(target) <= fabsf(current);
    if (braking) return current + (target - current) * alpha;

    // Paso máximo por ciclo = max_step / duty; se alcanza en GAIT_RAMP_CYCLES ciclos.
    float max_delta = (max_step / duty) * k / GAIT_RAMP_CYCLES;
    return current + scalar_clamp(target - current, -max_delta, max_delta);
}

/* --------------------------------------------------------------------------
 * Inicialización y configuración
 * -------------------------------------------------------------------------- */

void gait_engine_init(gait_engine_t *g, gait_mode_t mode, float period_s, float step_h, const vec3 neutral_body[LEG_COUNT])
{
    memset(g, 0, sizeof(gait_engine_t));
    for (int i = 0; i < LEG_COUNT; i++) {
        g->neutral_body[i] = neutral_body[i];
        g->foot_body[i] = neutral_body[i];
    }
    gait_engine_configure(g, mode, period_s, step_h);
}

void gait_engine_configure(gait_engine_t *g, gait_mode_t mode, float period_s, float step_h)
{
    if (!g) return;
    if (mode < GAIT_MODE_TRIPOD || mode > GAIT_MODE_AMBLE) mode = GAIT_MODE_TRIPOD;

    bool mode_changed = (mode != g->mode);
    g->mode = mode;
    g->duty = GAIT_DUTY[(int)mode - (int)GAIT_MODE_TRIPOD];
    g->period_s = (period_s >= 0.6f) ? period_s : 0.6f;
    g->step_h = scalar_clamp(step_h, 0.0f, GAIT_MAX_STEP_HEIGHT_MM);

    if (mode_changed) {
        g->phase = 0.0f;
        g->stride_x = 0.0f;
        g->stride_y = 0.0f;
        g->stride_yaw = 0.0f;
        g->target_stride_x = 0.0f;
        g->target_stride_y = 0.0f;
        g->target_stride_yaw = 0.0f;
        for (int i = 0; i < LEG_COUNT; i++) {
            g->was_swing[i] = false;
            g->swing_start_traj[i] = g->neutral_body[i];
            g->swing_end_traj[i] = g->neutral_body[i];
            g->swing_height[i] = 0.0f;
        }
    }
}

/* --------------------------------------------------------------------------
 * Comandos de movimiento
 * -------------------------------------------------------------------------- */

void gait_engine_set_command(gait_engine_t *g, float stride_x, float stride_y, float stride_yaw)
{
    float demand = fabsf(g->duty * stride_x) / GAIT_MAX_STEP_X_MM + fabsf(g->duty * stride_y) / GAIT_MAX_STEP_Y_MM + fabsf(g->duty * stride_yaw) / GAIT_MAX_STEP_YAW_RAD;
    float scale = (demand > 1.0f) ? 1.0f / demand : 1.0f;

    g->target_stride_x = stride_x * scale;
    g->target_stride_y = stride_y * scale;
    g->target_stride_yaw = stride_yaw * scale;
}

void gait_engine_halt(gait_engine_t *g)
{
    if (!g) return;
    g->stride_x = 0.0f;
    g->stride_y = 0.0f;
    g->stride_yaw = 0.0f;
}

/* --------------------------------------------------------------------------
 * Actualización cíclica
 * -------------------------------------------------------------------------- */

void gait_engine_step(gait_engine_t *g, body_kinematics_t *bk, float dt_s)
{
    if (dt_s <= 0.0f) return;

    /* 1. Rampa del comando e integración de la trayectoria. */

    // Fracción del ciclo transcurrida durante este tick.
    float k = dt_s / g->period_s;

    // Rampa al acelerar o invertir el sentido; filtro de 0,2 s al frenar.
    float alpha = scalar_clamp(5.0f * dt_s, 0.0f, 1.0f);
    g->stride_x = ramp_stride(g->stride_x, g->target_stride_x, GAIT_MAX_STEP_X_MM, g->duty, k, alpha);
    g->stride_y = ramp_stride(g->stride_y, g->target_stride_y, GAIT_MAX_STEP_Y_MM, g->duty, k, alpha);
    g->stride_yaw = ramp_stride(g->stride_yaw, g->target_stride_yaw, GAIT_MAX_STEP_YAW_RAD, g->duty, k, alpha);

    float dx = g->stride_x * k;
    float dy = g->stride_y * k;
    float dyaw = g->stride_yaw * k;
    body_kinematics_integrate_gait_delta(bk, dx, dy, dyaw);

    /* 2. Movimiento inverso necesario para mantener los apoyos fijos. */

    se3 qbody_inverse = rigid_transform_identity();
    if (g->have_prev) {
        // Q_body = inverse(T_gb_prev) * T_gb_curr.
        se3 qbody = body_kinematics_compute_qbody(&g->bk_prev, bk);
        qbody_inverse = rigid_transform_inverse(&qbody);
    }

    /* 3. Avance del oscilador maestro. */

    g->phase = wrap01(g->phase + k);

    /* 4. Generación del objetivo cartesiano de cada pata. */

    for (int li = 0; li < LEG_COUNT; li++) {
        leg_id_t leg = (leg_id_t)li;
        float offset = GAIT_PHASE_OFFSETS[(int)g->mode - (int)GAIT_MODE_TRIPOD][leg];
        float ph = wrap01(g->phase + offset);
        bool is_swing = (ph >= g->duty);

        if (!is_swing) {
            /* Fase de apoyo: el pie permanece fijo respecto al mundo. */

            if (g->have_prev) {
                // p_body,curr = inverse(Q_body) * p_body,prev.
                g->foot_body[leg] = rigid_transform_apply_to_point(&qbody_inverse, g->foot_body[leg]);
            }
        } else {
            /* Fase de transferencia: el pie vuelve al siguiente apoyo. */

            // Progreso normalizado dentro del intervalo de transferencia.
            float u = scalar_clamp((ph - g->duty) / (1.0f - g->duty), 0.0f, 1.0f);

            if (!g->was_swing[leg]) {
                // Completar el último incremento de apoyo antes de fijar el despegue.
                if (g->have_prev) g->foot_body[leg] = rigid_transform_apply_to_point(&qbody_inverse, g->foot_body[leg]);

                // El inicio es la posición real del pie expresada respecto a T_tr.
                g->swing_start_traj[leg] = rigid_transform_apply_to_point(&bk->T_body, g->foot_body[leg]);

                // El destino se calcula una sola vez con el paso efectivo del despegue.
                float step_x = g->stride_x * g->duty;
                float step_y = g->stride_y * g->duty;
                float step_yaw = g->stride_yaw * g->duty;

                // T_step,i y R_gait,i sitúan el próximo apoyo en el extremo delantero.
                se3 T_step_i = rigid_transform_identity();
                T_step_i.translation = vector3(0.5f * step_x, 0.5f * step_y, 0.0f);
                se3 R_gait_i = rigid_transform_identity();
                R_gait_i.rotation = rotation_matrix_z(0.5f * step_yaw);
                se3 swing_end_transform = rigid_transform_compose(&R_gait_i, &T_step_i);
                g->swing_end_traj[leg] = rigid_transform_apply_to_point(&swing_end_transform, g->neutral_body[leg]);

                // Elevar según el recorrido pendiente, incluso al volver a neutral en STOP.
                float swing_dx = g->swing_end_traj[leg].x - g->swing_start_traj[leg].x;
                float swing_dy = g->swing_end_traj[leg].y - g->swing_start_traj[leg].y;
                float swing_distance = sqrtf(scalar_square(swing_dx) + scalar_square(swing_dy));
                g->swing_height[leg] = g->step_h * scalar_clamp(swing_distance / 10.0f, 0.0f, 1.0f);
            }

            // Interpolación horizontal suave entre el despegue real y el destino fijado.
            float s = scalar_square(u) * (3.0f - 2.0f * u);
            vec3 p_swing_traj_body = vector3(g->swing_start_traj[leg].x + (g->swing_end_traj[leg].x - g->swing_start_traj[leg].x) * s, g->swing_start_traj[leg].y + (g->swing_end_traj[leg].y - g->swing_start_traj[leg].y) * s, g->swing_start_traj[leg].z + (g->swing_end_traj[leg].z - g->swing_start_traj[leg].z) * s);

            // p_swing_world = T_tr,i * p_traj_body + delta_z(u) * Z_world.
            vec3 p_swing_world = rigid_transform_apply_to_point(&bk->T_tr, p_swing_traj_body);
            p_swing_world.z += g->swing_height[leg] * scalar_square(sinf((float)M_PI * u));

            // La IK consume objetivos expresados en el frame BODY actual.
            g->foot_body[leg] = body_kinematics_point_world_to_body(bk, p_swing_world);
        }

        // Permite detectar las transiciones stance -> swing del siguiente tick.
        g->was_swing[leg] = is_swing;
    }

    // El estado actual se convierte en la referencia del siguiente tick.
    g->bk_prev = *bk;
    g->have_prev = true;
}
