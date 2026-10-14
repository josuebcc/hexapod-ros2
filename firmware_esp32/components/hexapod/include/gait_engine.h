/**
 * @file gait_engine.h
 * @brief API del generador de fases y trayectorias de marcha.
 * @ingroup group_gait_engine
 */

#pragma once

#include <stdbool.h>

#include "body_kinematics.h"
#include "leg_config.h"
#include "leg_kinematics.h"
#include "matrix_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_gait_engine gait_engine
 * @brief Coordinación temporal de las patas y generación de objetivos cartesianos.
 *
 * El módulo implementa las marchas Trípode, Ripple y Amble mediante una fase
 * global normalizada y un desfase fijo por pata. Para cada extremidad distingue
 * dos intervalos:
 *
 * - En apoyo (`stance`), aplica el movimiento inverso del cuerpo para mantener
 *   el pie fijo respecto al mundo.
 * - En transferencia (`swing`), interpola entre la posición real de despegue y
 *   el siguiente apoyo mediante un perfil horizontal cúbico y un perfil
 *   vertical sinusoidal cuadrático.
 *
 * Los comandos se expresan como desplazamientos por ciclo completo (`stride`).
 * El paso efectivo recuperado durante el vuelo se calcula internamente como
 * `step = duty * stride`. Las distancias se expresan en milímetros, los
 * ángulos en radianes y los tiempos en segundos.
 *
 * @par Código fuente
 * Declaración en gait_engine.h e implementación en gait_engine.cpp.
 *
 * @see group_body_kinematics
 * @see group_leg_kinematics
 * @see group_matrix_math
 * @{
 */

/* --------------------------------------------------------------------------
 * Tipos públicos
 * -------------------------------------------------------------------------- */

/** @name Tipos públicos
 *  @{ */

/**
 * @brief Estado completo del generador de marcha.
 *
 * Conserva la configuración del patrón, el oscilador global, los comandos
 * suavizados, los objetivos cartesianos de las patas y el estado cinemático
 * anterior necesario para calcular el movimiento relativo del cuerpo. Para
 * cada pata almacena además la trayectoria de swing fijada al despegar, de modo
 * que un cambio posterior del comando no modifica un vuelo ya iniciado.
 */
typedef struct {
    /* Configuración de la marcha. */
    gait_mode_t mode; /**< Patrón activo: Trípode, Ripple o Amble. */
    float period_s;   /**< Duración de un ciclo completo, en s. */
    float duty;       /**< Fracción del ciclo en apoyo, determinada por `mode`. */
    float step_h;     /**< Altura máxima configurada para el vuelo, en mm. */

    /* Oscilador maestro. */
    float phase; /**< Fase global normalizada en el intervalo [0, 1). */

    /* Desplazamiento por ciclo aplicado tras el suavizado. */
    float stride_x;   /**< Desplazamiento longitudinal actual, en mm/ciclo. */
    float stride_y;   /**< Desplazamiento lateral actual, en mm/ciclo. */
    float stride_yaw; /**< Giro actual alrededor de Z, en rad/ciclo. */

    /* Desplazamiento por ciclo solicitado. */
    float target_stride_x;   /**< Objetivo longitudinal, en mm/ciclo. */
    float target_stride_y;   /**< Objetivo lateral, en mm/ciclo. */
    float target_stride_yaw; /**< Objetivo angular, en rad/ciclo. */

    /* Geometría y salida cartesiana. */
    vec3 neutral_body[LEG_COUNT]; /**< Posiciones neutrales expresadas en BODY, en mm. */
    vec3 foot_body[LEG_COUNT];    /**< Objetivos actuales para la IK en BODY, en mm. */

    /* Trayectorias de transferencia fijadas en el instante de despegue. */
    bool was_swing[LEG_COUNT];        /**< true si la pata estaba en swing en el tick anterior. */
    vec3 swing_start_traj[LEG_COUNT]; /**< Despegue real respecto al frame de trayectoria, en mm. */
    vec3 swing_end_traj[LEG_COUNT];   /**< Aterrizaje fijado respecto al frame de trayectoria, en mm. */
    float swing_height[LEG_COUNT];    /**< Altura efectiva fijada al despegar, en mm. */

    /* Memoria para el movimiento relativo del cuerpo. */
    body_kinematics_t bk_prev; /**< Estado cinemático del tick anterior. */
    bool have_prev;            /**< true si `bk_prev` contiene un estado válido. */
} gait_engine_t;

/** @} */

/* --------------------------------------------------------------------------
 * Inicialización y configuración
 * -------------------------------------------------------------------------- */

/** @name Inicialización y configuración
 *  @{ */

/**
 * @brief Inicializa el generador de marcha.
 *
 * Reinicia el estado, copia las posiciones neutrales, inicializa con ellas los
 * objetivos cartesianos y aplica la configuración indicada. Un periodo menor
 * que 0,6 s se sustituye por 0,6 s y la altura se limita a 60 mm.
 * `GAIT_MODE_STOP` y los valores inválidos seleccionan Trípode, ya que STOP no
 * constituye un patrón interno.
 *
 * @param[out] g Generador que se inicializará.
 * @param[in] mode Modo de marcha inicial.
 * @param[in] period_s Duración inicial del ciclo, en s.
 * @param[in] step_h Altura máxima inicial del vuelo, en mm.
 * @param[in] neutral_body Posiciones neutrales de las seis patas en BODY, en mm.
 */
void gait_engine_init(gait_engine_t *g, gait_mode_t mode, float period_s, float step_h, const vec3 neutral_body[LEG_COUNT]);

/**
 * @brief Actualiza los parámetros dinámicos de la marcha.
 *
 * Configura el modo, el periodo y la altura de vuelo. Si cambia el modo,
 * reinicia la fase y los comandos de movimiento, pero conserva los objetivos
 * cartesianos y la pose anterior del cuerpo. `GAIT_MODE_STOP` y los valores
 * inválidos seleccionan Trípode.
 *
 * @param[in,out] g Generador que se actualizará.
 * @param[in] mode Modo de marcha.
 * @param[in] period_s Duración del ciclo, en s.
 * @param[in] step_h Altura máxima del vuelo, en mm.
 */
void gait_engine_configure(gait_engine_t *g, gait_mode_t mode, float period_s, float step_h);

/** @} */

/* --------------------------------------------------------------------------
 * Comandos de movimiento
 * -------------------------------------------------------------------------- */

/** @name Comandos de movimiento
 *  @{ */

/**
 * @brief Establece el desplazamiento objetivo por ciclo completo.
 *
 * Los valores se almacenan como objetivos y se aproximan progresivamente en
 * @ref gait_engine_step. Antes de almacenarlos, la función calcula el paso
 * efectivo `step = duty * stride` y aplica una restricción combinada:
 *
 * `|step_x| / 120 + |step_y| / 50 + |step_yaw| / 0.35 <= 1`.
 *
 * Si se supera el límite, las tres componentes se multiplican por un mismo
 * factor para conservar la dirección del movimiento solicitado.
 *
 * @param[in,out] g Generador que recibirá el comando.
 * @param[in] stride_x Desplazamiento longitudinal, en mm/ciclo.
 * @param[in] stride_y Desplazamiento lateral, en mm/ciclo.
 * @param[in] stride_yaw Giro alrededor de Z, en rad/ciclo.
 */
void gait_engine_set_command(gait_engine_t *g, float stride_x, float stride_y, float stride_yaw);

/**
 * @brief Anula el desplazamiento aplicado conservando el objetivo.
 *
 * Pone a cero `stride_x`, `stride_y` y `stride_yaw` sin modificar la fase ni
 * los objetivos. En los ticks siguientes la rampa de @ref gait_engine_step
 * vuelve a acercar el desplazamiento al objetivo vigente. Se utiliza cuando la
 * cinemática inversa rechaza un tick: con un objetivo nulo, los pies en apoyo
 * dejan de alejarse de su alcance y la marcha puede reanudarse.
 *
 * @param[in,out] g Generador cuyo desplazamiento se anulará.
 */
void gait_engine_halt(gait_engine_t *g);

/** @} */

/* --------------------------------------------------------------------------
 * Actualización cíclica
 * -------------------------------------------------------------------------- */

/** @name Actualización cíclica
 *  @{ */

/**
 * @brief Ejecuta un tick del generador de marcha.
 *
 * Aproxima el `stride` a su objetivo, integra el incremento comandado en
 * `T_tr`, calcula el movimiento relativo del cuerpo, avanza la fase global y
 * actualiza `foot_body` para todas las patas. Al acelerar o invertir el sentido,
 * el paso varía como máximo su valor máximo por ciclo; al frenar se aplica un
 * filtro de primer orden de constante 0,2 s. Al comenzar cada swing fija el punto real
 * de despegue, el destino y la altura; los cambios posteriores del comando se
 * aplican a esa pata en su siguiente swing.
 *
 * La función integra también en `bk->T_tr` el incremento odométrico asociado al
 * comando.
 *
 * Si @p dt_s es menor o igual que cero, la función no modifica el estado.
 *
 * @param[in,out] g Generador de marcha.
 * @param[in,out] bk Estado cinemático cuya trayectoria `T_tr` se integrará.
 * @param[in] dt_s Tiempo transcurrido desde el tick anterior, en s.
 */
void gait_engine_step(gait_engine_t *g, body_kinematics_t *bk, float dt_s);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
