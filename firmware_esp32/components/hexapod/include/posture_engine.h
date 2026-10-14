/**
 * @file posture_engine.h
 * @brief API del suavizado temporal de la postura del cuerpo.
 * @ingroup group_posture_engine
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_posture_engine posture_engine
 * @brief Interpolación suave de las consignas de postura del chasis.
 *
 * El módulo mantiene una transición independiente para roll, pitch, yaw y las
 * tres traslaciones del cuerpo. Cuando cambia un objetivo, la nueva trayectoria
 * parte del valor actual y utiliza un perfil quíntico con velocidad y
 * aceleración nulas en ambos extremos.
 *
 * @par Código fuente
 * Declaración en posture_engine.h e implementación en posture_engine.cpp.
 *
 * @see group_body_kinematics
 * @see group_locomotion_task
 * @{
 */

/* --------------------------------------------------------------------------
 * Tipos públicos
 * -------------------------------------------------------------------------- */

/** @name Tipos públicos
 *  @{ */

/**
 * @brief Estado de la transición suave de una magnitud escalar.
 */
typedef struct {
    float start_val;              /**< Valor inicial de la transición. */
    float target_val;             /**< Objetivo vigente. */
    float current_val;            /**< Valor interpolado actual. */
    float tiempo_transcurrido;    /**< Tiempo avanzado desde el inicio, en s. */
    float duration;               /**< Duración total de la transición, en s. */
} poly_transition_t;

/**
 * @brief Estado completo del motor de postura.
 */
typedef struct {
    float default_transition_time; /**< Duración común de las transiciones, en s. */

    poly_transition_t t_roll;      /**< Transición de roll. */
    poly_transition_t t_pitch;     /**< Transición de pitch. */
    poly_transition_t t_yaw;       /**< Transición de yaw. */
    poly_transition_t t_x;         /**< Transición de la traslación X. */
    poly_transition_t t_y;         /**< Transición de la traslación Y. */
    poly_transition_t t_z;         /**< Transición de la traslación Z. */

    float curr_roll;               /**< Roll suavizado actual, en rad. */
    float curr_pitch;              /**< Pitch suavizado actual, en rad. */
    float curr_yaw;                /**< Yaw suavizado actual, en rad. */
    float curr_x;                  /**< Traslación X suavizada actual, en mm. */
    float curr_y;                  /**< Traslación Y suavizada actual, en mm. */
    float curr_z;                  /**< Traslación Z suavizada actual, en mm. */
} posture_engine_t;

/** @} */

/* --------------------------------------------------------------------------
 * Inicialización y configuración
 * -------------------------------------------------------------------------- */

/** @name Inicialización y configuración
 *  @{ */

/**
 * @brief Inicializa el motor con postura nula y transiciones finalizadas.
 *
 * @param[out] eng Motor que se inicializará.
 * @param[in] transition_time_s Duración inicial de las transiciones, en s.
 */
void posture_engine_init(posture_engine_t *eng, float transition_time_s);

/**
 * @brief Actualiza la duración común de las transiciones.
 *
 * Los valores inferiores a 0,1 s se sustituyen por 0,1 s. La nueva duración
 * se aplica también a las transiciones que se encuentren activas.
 *
 * @param[in,out] eng Motor que se configurará.
 * @param[in] time_s Nueva duración, en s.
 */
void posture_engine_set_transition_time(posture_engine_t *eng, float time_s);

/** @} */

/* --------------------------------------------------------------------------
 * Objetivo y actualización
 * -------------------------------------------------------------------------- */

/** @name Objetivo y actualización
 *  @{ */

/**
 * @brief Establece la postura objetivo del cuerpo.
 *
 * Cada componente que cambie más de 0,001 unidades inicia una nueva transición
 * desde su valor interpolado actual. Las componentes sin cambios continúan su
 * trayectoria vigente.
 *
 * @param[in,out] eng Motor de postura.
 * @param[in] target_roll Objetivo de roll, en rad.
 * @param[in] target_pitch Objetivo de pitch, en rad.
 * @param[in] target_yaw Objetivo de yaw, en rad.
 * @param[in] target_x Objetivo de traslación X, en mm.
 * @param[in] target_y Objetivo de traslación Y, en mm.
 * @param[in] target_z Objetivo de traslación Z, en mm.
 */
void posture_engine_set_target(posture_engine_t *eng, float target_roll, float target_pitch, float target_yaw, float target_x, float target_y, float target_z);

/**
 * @brief Avanza las seis transiciones y actualiza las salidas suavizadas.
 *
 * @param[in,out] eng Motor de postura.
 * @param[in] dt_s Tiempo transcurrido desde la actualización anterior, en s.
 */
void posture_engine_step(posture_engine_t *eng, float dt_s);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
