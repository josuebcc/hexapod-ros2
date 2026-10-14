/**
 * @file leg_kinematics.h
 * @brief API de cinemática inversa y adaptación angular de las patas.
 */

#pragma once

#include <math.h>

#include "leg_config.h"
#include "matrix_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_leg_kinematics leg_kinematics
 * @brief Cambio de coordenadas, IK geométrica y consignas articulares.
 *
 * El módulo transforma un objetivo cartesiano desde el frame BODY al frame
 * local LEG, resuelve la cinemática inversa de la cadena coxa--fémur--tibia y
 * adapta los ángulos obtenidos al sistema de los servomotores. Los cálculos geométricos
 * emplean milímetros y radianes. La función de alto nivel @ref leg_compute_deg devuelve
 * las consignas finales en grados y aplica los offsets, trims, signos y límites definidos en @ref group_leg_config.
 *
 * @par Código fuente
 * Declaración en leg_kinematics.h e implementación en leg_kinematics.cpp.
 *
 * @see group_leg_config
 * @see group_matrix_math
 * @{
 */

/* --------------------------------------------------------------------------
 * Tipos públicos
 * -------------------------------------------------------------------------- */

/** @name Tipos públicos
 *  @{ */

/**
 * @brief Solución geométrica de la IK de una pata.
 */
typedef struct {
    float theta; /**< Ángulo de la coxa, en rad. */
    float phi;   /**< Ángulo del fémur, en rad. */
    float psi;   /**< Ángulo de la tibia, en rad. */
} leg_angles;

/** @} */

/* --------------------------------------------------------------------------
 * Acceso a la configuración
 * -------------------------------------------------------------------------- */

/** @name Acceso a la configuración
 *  @{ */

/**
 * @brief Obtiene la configuración de una pata.
 *
 * @param[in] id Identificador de la pata.
 * @return Puntero a la configuración constante o `NULL` si @p id no es válido.
 */
const leg_cfg_t* leg_get_cfg(leg_id_t id);

/** @} */

/* --------------------------------------------------------------------------
 * Cambio de coordenadas
 * -------------------------------------------------------------------------- */

/** @name Cambio de coordenadas
 *  @{ */

/**
 * @brief Transforma un punto del frame BODY al frame local LEG.
 *
 * Aplica `p_leg = [x_body + xj, -y_body + yj, -z_body]`.
 *
 * @param[in] p_body Punto expresado en BODY, en mm.
 * @param[in] p Geometría y anclaje de la pata.
 * @return Punto expresado en el frame local de la pata, en mm.
 */
static inline vec3 body_to_leg(vec3 p_body, const leg_params *p)
{
    vec3 p_leg;
    p_leg.x = p_body.x + p->xj;
    p_leg.y = -p_body.y + p->yj;
    p_leg.z = -p_body.z;
    return p_leg;
}

/** @} */

/* --------------------------------------------------------------------------
 * Cinemática inversa
 * -------------------------------------------------------------------------- */

/** @name Cinemática inversa
 *  @{ */

/**
 * @brief Resuelve la IK geométrica de una pata.
 *
 * Calcula los ángulos articulares sin aplicar offsets, trims, signos ni
 * límites de servomotor.
 *
 * @param[in] p_leg Objetivo cartesiano expresado en LEG, en mm.
 * @param[in] par Geometría de la pata.
 * @param[out] out Ángulos de coxa, fémur y tibia, en rad.
 * @return true si el objetivo pertenece al espacio de trabajo geométrico;
 *         false si los argumentos son nulos o el objetivo no es alcanzable.
 */
bool leg_ik(vec3 p_leg, const leg_params *par, leg_angles *out);

/**
 * @brief Calcula las consignas angulares finales de una pata.
 *
 * Ejecuta el cambio BODY -> LEG, resuelve la IK, convierte la solución a
 * grados y aplica `servo = offset + trim + signo * angulo_ik`. Finalmente,
 * satura cada articulación a los límites software configurados.
 *
 * @param[in] leg Configuración completa de la pata.
 * @param[in] xb Coordenada X del objetivo en BODY, en mm.
 * @param[in] yb Coordenada Y del objetivo en BODY, en mm.
 * @param[in] zb Coordenada Z del objetivo en BODY, en mm.
 * @param[out] out_deg3 Consignas en grados: coxa, fémur y tibia.
 * @return true si existe una solución geométrica; false si los argumentos son
 *         nulos o el objetivo no es alcanzable.
 * @note Un resultado true puede contener valores saturados por los límites
 *       software.
 */
bool leg_compute_deg(const leg_cfg_t *leg, float xb, float yb, float zb, float out_deg3[3]);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
