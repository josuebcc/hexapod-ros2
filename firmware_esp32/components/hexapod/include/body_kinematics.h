/**
 * @file body_kinematics.h
 * @brief API de cinemática y odometría del cuerpo del hexápodo.
 */

#pragma once

#include "matrix_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_body_kinematics body_kinematics
 * @brief Transformaciones del cuerpo, odometría y movimiento relativo entre poses.
 *
 * El módulo mantiene separadas la trayectoria acumulada del robot y la postura
 * local del chasis:
 *
 * - `T_tr` contiene la posición plana y la orientación yaw integradas por la
 *   marcha, además de la altura base del cuerpo.
 * - `T_body` contiene los desplazamientos y las rotaciones locales de postura.
 *
 * La pose completa del cuerpo respecto al mundo se obtiene mediante T_gb = T_tr * T_body
 *
 *
 * @par Código fuente
 * Declaración en body_kinematics.h e implementación en body_kinematics.cpp.
 *
 * @see group_matrix_math
 * @{
 */

/* --------------------------------------------------------------------------
 * Tipos públicos
 * -------------------------------------------------------------------------- */

/** @name Tipos públicos
 *  @{ */

/**
 * @brief Estado cinemático del cuerpo.
 *
 * Almacena por separado la transformación de trayectoria y la transformación
 * de postura cuya composición produce la pose completa World <- Body.
 */
typedef struct {
    se3 T_tr;    /**< Trayectoria y odometría acumuladas respecto al mundo. */
    se3 T_body;  /**< Postura instantánea respecto al frame de trayectoria. */
} body_kinematics_t;

/** @} */

/* --------------------------------------------------------------------------
 * Inicialización
 * -------------------------------------------------------------------------- */

/** @name Inicialización
 *  @{ */

/**
 * @brief Inicializa el estado cinemático del cuerpo.
 *
 * Establece `T_tr` sin rotación, con X e Y nulos y con la altura base indicada.
 * `T_body` se inicializa con la transformación identidad.
 *
 * @param[out] bk Estado cinemático que se inicializará.
 * @param[in] initial_z_mm Altura inicial del cuerpo respecto al suelo, en mm.
 */
void body_kinematics_init(body_kinematics_t *bk, float initial_z_mm);

/** @} */

/* --------------------------------------------------------------------------
 * Actualización del estado
 * -------------------------------------------------------------------------- */

/** @name Actualización del estado
 *  @{ */

/**
 * @brief Establece la postura local instantánea del cuerpo.
 *
 * Sustituye `T_body` por una transformación construida con la convención
 * Rz(yaw)Ry(pitch)Rx(roll). Esta operación no modifica la odometría acumulada
 * en `T_tr`.
 *
 * @param[in,out] bk Estado cinemático que se actualizará.
 * @param[in] roll Rotación local alrededor de X, en rad.
 * @param[in] pitch Rotación local alrededor de Y, en rad.
 * @param[in] yaw Rotación local alrededor de Z, en rad.
 * @param[in] x Desplazamiento local en X, en mm.
 * @param[in] y Desplazamiento local en Y, en mm.
 * @param[in] z Desplazamiento local respecto a la altura base, en mm.
 */
void body_kinematics_set_posture_rpy_xyz(body_kinematics_t *bk, float roll, float pitch, float yaw, float x, float y, float z);

/**
 * @brief Integra un incremento plano de la marcha en la odometría.
 *
 * T_tr_new = T_tr_old * Q
 *
 * @param[in,out] bk Estado cinemático que se actualizará.
 * @param[in] dx_body Incremento longitudinal local en X, en mm.
 * @param[in] dy_body Incremento lateral local en Y, en mm.
 * @param[in] dyaw Incremento de yaw alrededor de Z, en rad.
 */
void body_kinematics_integrate_gait_delta(body_kinematics_t *bk, float dx_body, float dy_body, float dyaw);

/** @} */

/* --------------------------------------------------------------------------
 * Pose y transformación de puntos
 * -------------------------------------------------------------------------- */

/** @name Pose y transformación de puntos
 *  @{ */

/**
 * @brief Obtiene la transformación completa World <- Body.
 *
 * Compone la trayectoria acumulada y la postura local como
 * `T_gb = T_tr * T_body`.
 *
 * @param[in] bk Estado cinemático del cuerpo.
 * @return Transformación que expresa el frame Body respecto al frame World.
 */
se3 body_kinematics_get_world_from_body(const body_kinematics_t *bk);

/**
 * @brief Transforma un punto del frame World al frame Body.
 *
 * Aplica `p_body = inverse(T_gb) * p_world`.
 *
 * @param[in] bk Estado cinemático del cuerpo.
 * @param[in] p_world Punto expresado en el frame World, en mm.
 * @return Punto expresado en el frame Body, en mm.
 */
vec3 body_kinematics_point_world_to_body(const body_kinematics_t *bk, vec3 p_world);

/** @} */

/* --------------------------------------------------------------------------
 * Movimiento relativo
 * -------------------------------------------------------------------------- */

/** @name Movimiento relativo
 *  @{ */

/**
 * @brief Calcula el movimiento relativo entre dos estados cinemáticos.
 *
 * Obtiene las poses completas de ambos estados y calcula
 * `Q_body = inverse(T_gb_prev) * T_gb_curr`.
 *
 * @param[in] bk_prev Estado cinemático del instante anterior.
 * @param[in] bk_curr Estado cinemático del instante actual.
 * @return Transformación Body_anterior <- Body_actual.
 */
se3 body_kinematics_compute_qbody(const body_kinematics_t *bk_prev, const body_kinematics_t *bk_curr);

/** @} */

/* --------------------------------------------------------------------------
 * Odometría
 * -------------------------------------------------------------------------- */

/** @name Odometría
 *  @{ */

/**
 * @brief Obtiene la odometría plana acumulada.
 *
 * Extrae X, Y y yaw exclusivamente de `T_tr`; la postura local almacenada en
 * `T_body` no forma parte de la odometría reportada. El yaw se obtiene mediante
 * `atan2` y queda normalizado en el intervalo [-pi, pi].
 *
 * @param[in] bk Estado cinemático del cuerpo.
 * @param[out] x Posición acumulada en X, en mm, o NULL.
 * @param[out] y Posición acumulada en Y, en mm, o NULL.
 * @param[out] yaw Orientación alrededor de Z normalizada en [-pi, pi], en rad,
 *                 o NULL.
 */
void body_kinematics_get_odometry(const body_kinematics_t *bk, float *x, float *y, float *yaw);

/**
 * @brief Reinicia la odometría y conserva la postura local.
 *
 * Sustituye `T_tr` por una transformación sin rotación y con X e Y nulos.
 * Conserva la altura base almacenada en `T_tr` y no modifica `T_body`.
 *
 * @param[in,out] bk Estado cinemático que se reiniciará.
 */
void body_kinematics_reset_odometry(body_kinematics_t *bk);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
