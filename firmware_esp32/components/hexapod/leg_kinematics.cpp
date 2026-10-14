/**
 * @file leg_kinematics.cpp
 * @brief Implementación de la cinemática inversa de las patas.
 */

#include "leg_kinematics.h"

#include "leg_config.h"

/* --------------------------------------------------------------------------
 * Acceso a la configuración
 * -------------------------------------------------------------------------- */

const leg_cfg_t* leg_get_cfg(leg_id_t id)
{
    if ((int)id < 0 || id >= LEG_COUNT) {
        return nullptr;
    }
    return &g_leg_cfgs[id];
}

/* --------------------------------------------------------------------------
 * Cinemática inversa geométrica
 * -------------------------------------------------------------------------- */

bool leg_ik(vec3 p_leg, const leg_params *par, leg_angles *out)
{
    if (!par || !out) {
        return false;
    }

    // Coordenadas del objetivo en el frame local LEG.
    float x = p_leg.x;
    float y = p_leg.y;
    float z = p_leg.z;

    // Distancia radial desde el eje del fémur, descontando la coxa.
    float trueX = sqrtf(scalar_square(x) + scalar_square(y)) - par->lc;

    // Distancia directa entre la articulación del fémur y el pie.
    float im = sqrtf(scalar_square(trueX) + scalar_square(z));

    // El triángulo fémur-tibia solo existe dentro de este intervalo.
    float maxR = par->lf + par->lt;
    float minR = fabsf(par->lf - par->lt);
    if (im < minR || im > maxR) {
        return false;
    }

    // Coxa: orientación horizontal corregida por el montaje de la pata.
    out->theta = -atan2f(y, x) - par->eps;
    // Se escoge la representación equivalente contenida en [-pi, pi].
    out->theta = atan2f(sinf(out->theta), cosf(out->theta));

    // Fémur: inclinación del objetivo y ley del coseno.
    float cos_phi =
        (scalar_square(par->lf) + scalar_square(im) - scalar_square(par->lt)) /
        (2.0f * im * par->lf);
    cos_phi = scalar_clamp(cos_phi, -1.0f, 1.0f);
    out->phi = (float)M_PI_2 - atan2f(trueX, z) - acosf(cos_phi);

    // Tibia: ángulo interno opuesto a la distancia fémur-pie.
    float cos_psi =
        (scalar_square(par->lf) + scalar_square(par->lt) - scalar_square(im)) /
        (2.0f * par->lf * par->lt);
    cos_psi = scalar_clamp(cos_psi, -1.0f, 1.0f);
    out->psi = (float)M_PI - acosf(cos_psi);

    return true;
}

/* --------------------------------------------------------------------------
 * Conversión a consignas de servomotor
 * -------------------------------------------------------------------------- */

bool leg_compute_deg(const leg_cfg_t *leg, float xb, float yb, float zb, float out_deg3[3])
{
    if (!leg || !out_deg3) {
        return false;
    }

    // 1. Cambio de coordenadas BODY -> LEG.
    vec3 p_leg = body_to_leg(vector3(xb, yb, zb), &leg->L);

    // 2. Resolución geométrica en radianes.
    leg_angles a;
    if (!leg_ik(p_leg, &leg->L, &a)) {
        return false;
    }

    // 3. Conversión de la solución a grados.
    float ik_coxa  = a.theta * (180.0f / M_PI);
    float ik_femur = a.phi * (180.0f / M_PI);
    float ik_tibia = a.psi * (180.0f / M_PI);

    // 4. Adaptación al cero y al sentido de cada servomotor.
    float coxa  = leg->off_coxa_deg  + leg->trim_coxa_deg  + leg->s_coxa  * ik_coxa;
    float femur = leg->off_femur_deg + leg->trim_femur_deg + leg->s_femur * ik_femur;
    float tibia = leg->off_tibia_deg + leg->trim_tibia_deg + leg->s_tibia * ik_tibia;

    // 5. Saturación software en el sistema angular del servomotor.
    coxa  = scalar_clamp(coxa,  leg->min_coxa_deg,  leg->max_coxa_deg);
    femur = scalar_clamp(femur, leg->min_femur_deg, leg->max_femur_deg);
    tibia = scalar_clamp(tibia, leg->min_tibia_deg, leg->max_tibia_deg);

    // Orden de salida esperado por locomotion_task.
    out_deg3[0] = coxa;
    out_deg3[1] = femur;
    out_deg3[2] = tibia;
    return true;
}
