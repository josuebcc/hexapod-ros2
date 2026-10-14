/**
 * @file leg_config.h
 * @brief Geometría, montaje y límites software de las patas del hexápodo.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_leg_config leg_config
 * @brief Configuración estática de las seis patas.
 *
 * El módulo reúne los parámetros que diferencian una pata de otra: IDs de los
 * actuadores, geometría, posición del anclaje, orientación, ajustes de montaje
 * y límites angulares software. El orden común del firmware es LT, LM, LB, RB,
 * RM y RT.
 *
 * Los límites se expresan en el sistema angular del servomotor, después de
 * aplicar el offset, el trim y el signo correspondientes. Hasta completar la
 * calibración mecánica se utiliza el recorrido general de 0--300 grados del
 * AX-12A.
 *
 * @par Código fuente
 * Declaración en leg_config.h y definición de la tabla en leg_config.cpp.
 *
 * @see group_leg_kinematics
 * @{
 */

/* --------------------------------------------------------------------------
 * Tipos públicos
 * -------------------------------------------------------------------------- */

/** @name Tipos públicos
 *  @{ */

/**
 * @brief Modos de funcionamiento de la locomoción.
 *
 * Este tipo es compartido por el controlador y el motor de marcha para evitar
 * conversiones entre enumeraciones equivalentes. `GAIT_MODE_STOP` representa
 * una orden de velocidad nula; el motor conserva internamente el último patrón
 * activo hasta que se solicita otra marcha.
 *
 * @see group_gait_engine
 */
typedef enum {
    GAIT_MODE_STOP = 0,    /**< Detención conservando el último patrón activo. */
    GAIT_MODE_TRIPOD = 1,  /**< Dos grupos alternos de tres patas. */
    GAIT_MODE_RIPPLE = 2,  /**< Transferencia secuencial de una pata. */
    GAIT_MODE_AMBLE = 3    /**< Tres grupos alternos de dos patas. */
} gait_mode_t;

/**
 * @brief Parámetros geométricos y anclaje de una pata.
 */
typedef struct {
    float lc;   /**< Longitud de la coxa, en mm. */
    float lf;   /**< Longitud del fémur, en mm. */
    float lt;   /**< Longitud de la tibia, en mm. */
    float xj;   /**< Traslación BODY -> LEG en X, en mm. */
    float yj;   /**< Traslación BODY -> LEG en Y, en mm. */
    float eps;  /**< Orientación fija de la coxa respecto al cuerpo, en rad. */
} leg_params;

/**
 * @brief Configuración completa de una pata.
 *
 * Agrupa la geometría y todos los parámetros necesarios para transformar los
 * ángulos de la IK en consignas de los servomotores.
 */
typedef struct {
    const char *name;  /**< Nombre identificativo de la pata. */

    uint8_t id_coxa;   /**< ID del servo de la coxa. */
    uint8_t id_femur;  /**< ID del servo del fémur. */
    uint8_t id_tibia;  /**< ID del servo de la tibia. */

    leg_params L;      /**< Geometría, anclaje y orientación de la pata. */

    float off_coxa_deg;   /**< Referencia angular del servo de coxa, en grados. */
    float off_femur_deg;  /**< Referencia angular del servo de fémur, en grados. */
    float off_tibia_deg;  /**< Referencia angular del servo de tibia, en grados. */

    float trim_coxa_deg;  /**< Corrección adicional de coxa, en grados. */
    float trim_femur_deg; /**< Corrección adicional de fémur, en grados. */
    float trim_tibia_deg; /**< Corrección adicional de tibia, en grados. */

    float s_coxa;   /**< Sentido de giro de la coxa: +1 o -1. */
    float s_femur;  /**< Sentido de giro del fémur: +1 o -1. */
    float s_tibia;  /**< Sentido de giro de la tibia: +1 o -1. */

    float min_coxa_deg;   /**< Límite software mínimo de coxa, en grados. */
    float max_coxa_deg;   /**< Límite software máximo de coxa, en grados. */
    float min_femur_deg;  /**< Límite software mínimo de fémur, en grados. */
    float max_femur_deg;  /**< Límite software máximo de fémur, en grados. */
    float min_tibia_deg;  /**< Límite software mínimo de tibia, en grados. */
    float max_tibia_deg;  /**< Límite software máximo de tibia, en grados. */
} leg_cfg_t;

/**
 * @brief Identificadores de las patas en el orden fijo del motor.
 */
typedef enum {
    LEGID_LT = 0,  /**< Pata izquierda delantera. */
    LEGID_LM,      /**< Pata izquierda central. */
    LEGID_LB,      /**< Pata izquierda trasera. */
    LEGID_RB,      /**< Pata derecha trasera. */
    LEGID_RM,      /**< Pata derecha central. */
    LEGID_RT,      /**< Pata derecha delantera. */
    LEG_COUNT = 6  /**< Número total de patas. */
} leg_id_t;

/** @} */

/* --------------------------------------------------------------------------
 * Tabla de configuración
 * -------------------------------------------------------------------------- */

/** @name Tabla de configuración
 *  @{ */

/**
 * @brief Configuración canónica de las seis patas.
 *
 * Se indexa mediante @ref leg_id_t.
 *
 * @warning Los límites actuales de 0--300 grados representan el recorrido
 *          general del AX-12A, no una calibración mecánica del montaje.
 */
extern const leg_cfg_t g_leg_cfgs[LEG_COUNT];

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
