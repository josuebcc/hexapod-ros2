/**
 * @file posture_engine.cpp
 * @brief Implementación del suavizado polinómico de postura.
 * @ingroup group_posture_engine
 */

#include "posture_engine.h"

#include <math.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Funciones auxiliares
 * -------------------------------------------------------------------------- */

/**
 * @brief Evalúa el perfil quíntico 6t^5 - 15t^4 + 10t^3.
 *
 * @param[in] t Progreso normalizado.
 * @return Factor suavizado limitado al intervalo [0, 1].
 */
static inline float smootherstep(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

/**
 * @brief Inicializa una transición escalar en el valor cero.
 *
 * El tiempo comienza igual a la duración para representar una transición ya
 * finalizada hasta que se reciba el primer objetivo.
 *
 * @param[out] tr Transición que se inicializará.
 * @param[in] duration Duración inicial, en s.
 */
static void init_transition(poly_transition_t *tr, float duration)
{
    tr->start_val = 0.0f;
    tr->target_val = 0.0f;
    tr->current_val = 0.0f;
    tr->tiempo_transcurrido = duration;
    tr->duration = duration;
}

/**
 * @brief Reinicia una transición cuando cambia su objetivo.
 *
 * La tolerancia evita reinicios causados por diferencias numéricas pequeñas.
 *
 * @param[in,out] tr Transición que se comprobará.
 * @param[in] new_target Nuevo objetivo.
 */
static void check_and_update_target(poly_transition_t *tr, float new_target)
{
    if (fabsf(new_target - tr->target_val) > 0.001f) {
        tr->start_val = tr->current_val;
        tr->target_val = new_target;
        tr->tiempo_transcurrido = 0.0f;
    }
}

/**
 * @brief Avanza una transición escalar.
 *
 * @param[in,out] tr Transición que se actualizará.
 * @param[in] dt_s Incremento temporal, en s.
 * @return Valor interpolado después de la actualización.
 */
static float update_transition_step(poly_transition_t *tr, float dt_s)
{
    if (tr->tiempo_transcurrido < tr->duration) {
        tr->tiempo_transcurrido += dt_s;
        if (tr->tiempo_transcurrido >= tr->duration) {
            tr->tiempo_transcurrido = tr->duration;
            tr->current_val = tr->target_val;
        } else {
            float t = tr->tiempo_transcurrido / tr->duration;
            tr->current_val = tr->start_val + (tr->target_val - tr->start_val) * smootherstep(t);
        }
    } else {
        tr->current_val = tr->target_val;
    }
    return tr->current_val;
}

/* --------------------------------------------------------------------------
 * Inicialización y configuración
 * -------------------------------------------------------------------------- */

void posture_engine_init(posture_engine_t *eng, float transition_time_s)
{
    memset(eng, 0, sizeof(posture_engine_t));
    eng->default_transition_time = transition_time_s;

    init_transition(&eng->t_roll, transition_time_s);
    init_transition(&eng->t_pitch, transition_time_s);
    init_transition(&eng->t_yaw, transition_time_s);
    init_transition(&eng->t_x, transition_time_s);
    init_transition(&eng->t_y, transition_time_s);
    init_transition(&eng->t_z, transition_time_s);
}

void posture_engine_set_transition_time(posture_engine_t *eng, float time_s)
{
    if (time_s < 0.1f) time_s = 0.1f;

    eng->default_transition_time = time_s;
    eng->t_roll.duration = time_s;
    eng->t_pitch.duration = time_s;
    eng->t_yaw.duration = time_s;
    eng->t_x.duration = time_s;
    eng->t_y.duration = time_s;
    eng->t_z.duration = time_s;
}

/* --------------------------------------------------------------------------
 * Objetivo y actualización
 * -------------------------------------------------------------------------- */

void posture_engine_set_target(posture_engine_t *eng, float roll, float pitch, float yaw, float x, float y, float z)
{
    check_and_update_target(&eng->t_roll, roll);
    check_and_update_target(&eng->t_pitch, pitch);
    check_and_update_target(&eng->t_yaw, yaw);
    check_and_update_target(&eng->t_x, x);
    check_and_update_target(&eng->t_y, y);
    check_and_update_target(&eng->t_z, z);
}

void posture_engine_step(posture_engine_t *eng, float dt_s)
{
    eng->curr_roll = update_transition_step(&eng->t_roll, dt_s);
    eng->curr_pitch = update_transition_step(&eng->t_pitch, dt_s);
    eng->curr_yaw = update_transition_step(&eng->t_yaw, dt_s);
    eng->curr_x = update_transition_step(&eng->t_x, dt_s);
    eng->curr_y = update_transition_step(&eng->t_y, dt_s);
    eng->curr_z = update_transition_step(&eng->t_z, dt_s);
}
