/**
 * @file leg_config.cpp
 * @brief Definición de la configuración estática de las seis patas.
 *
 * Las longitudes y los anclajes se expresan en milímetros; `eps`, en radianes;
 * y los offsets, trims y límites, en grados. Los límites 0--300 grados son
 * provisionales hasta realizar la calibración mecánica del montaje.
 */

#include "leg_config.h"

#include <math.h>

/* --------------------------------------------------------------------------
 * Tabla canónica de patas
 * -------------------------------------------------------------------------- */

const leg_cfg_t g_leg_cfgs[LEG_COUNT] = {
    /* Izquierda delantera (LT). */
    {
        .name = "LEG_IZQUIERDA DELANTERA",
        .id_coxa = 1, .id_femur = 3, .id_tibia = 5,
        .L = { .lc=52.0f, .lf=66.0f, .lt=135.0f, .xj=-120.0f, .yj=60.0f, .eps=(float)M_PI_4 },
        .off_coxa_deg=150.0f, .off_femur_deg=150.0f, .off_tibia_deg=150.0f,
        .trim_coxa_deg=0.0f,  .trim_femur_deg=0.0f,  .trim_tibia_deg=-45.0f,
        .s_coxa=+1.0f, .s_femur=+1.0f, .s_tibia=+1.0f,
        .min_coxa_deg=0.0f, .max_coxa_deg=300.0f,
        .min_femur_deg=0.0f, .max_femur_deg=300.0f,
        .min_tibia_deg=0.0f, .max_tibia_deg=300.0f,
    },
    /* Izquierda central (LM). */
    {
        .name = "LEG_IZQUIERDA MEDIO",
        .id_coxa = 13, .id_femur = 15, .id_tibia = 17,
        .L = { .lc=52.0f, .lf=66.0f, .lt=135.0f, .xj=0.0f, .yj=100.0f, .eps=(float)M_PI_2 },
        .off_coxa_deg=150.0f, .off_femur_deg=150.0f, .off_tibia_deg=150.0f,
        .trim_coxa_deg=0.0f,  .trim_femur_deg=0.0f,  .trim_tibia_deg=-45.0f,
        .s_coxa=+1.0f, .s_femur=+1.0f, .s_tibia=+1.0f,
        .min_coxa_deg=0.0f, .max_coxa_deg=300.0f,
        .min_femur_deg=0.0f, .max_femur_deg=300.0f,
        .min_tibia_deg=0.0f, .max_tibia_deg=300.0f,
    },
    /* Izquierda trasera (LB). */
    {
        .name = "LEG_IZQUIERDA ATRAS",
        .id_coxa = 7, .id_femur = 9, .id_tibia = 11,
        .L = { .lc=52.0f, .lf=66.0f, .lt=135.0f, .xj=120.0f, .yj=60.0f, .eps=(float)M_3PI_4 },
        .off_coxa_deg=150.0f, .off_femur_deg=150.0f, .off_tibia_deg=150.0f,
        .trim_coxa_deg=0.0f,  .trim_femur_deg=0.0f,  .trim_tibia_deg=-45.0f,
        .s_coxa=+1.0f, .s_femur=+1.0f, .s_tibia=+1.0f,
        .min_coxa_deg=0.0f, .max_coxa_deg=300.0f,
        .min_femur_deg=0.0f, .max_femur_deg=300.0f,
        .min_tibia_deg=0.0f, .max_tibia_deg=300.0f,
    },
    /* Derecha trasera (RB). */
    {
        .name = "LEG DERECHA ATRAS",
        .id_coxa = 8, .id_femur = 10, .id_tibia = 12,
        .L = { .lc=52.0f, .lf=66.0f, .lt=135.0f, .xj=120.0f, .yj=-60.0f, .eps=-(float)M_3PI_4 },
        .off_coxa_deg=150.0f, .off_femur_deg=150.0f, .off_tibia_deg=150.0f,
        .trim_coxa_deg=0.0f,  .trim_femur_deg=0.0f,  .trim_tibia_deg=45.0f,
        .s_coxa=+1.0f, .s_femur=-1.0f, .s_tibia=-1.0f,
        .min_coxa_deg=0.0f, .max_coxa_deg=300.0f,
        .min_femur_deg=0.0f, .max_femur_deg=300.0f,
        .min_tibia_deg=0.0f, .max_tibia_deg=300.0f,
    },
    /* Derecha central (RM). */
    {
        .name = "LEG DERECHA MEDIO",
        .id_coxa = 14, .id_femur = 16, .id_tibia = 18,
        .L = { .lc=52.0f, .lf=66.0f, .lt=135.0f, .xj=0.0f, .yj=-100.0f, .eps=-(float)M_PI_2 },
        .off_coxa_deg=150.0f, .off_femur_deg=150.0f, .off_tibia_deg=150.0f,
        .trim_coxa_deg=0.0f,  .trim_femur_deg=0.0f,  .trim_tibia_deg=45.0f,
        .s_coxa=+1.0f, .s_femur=-1.0f, .s_tibia=-1.0f,
        .min_coxa_deg=0.0f, .max_coxa_deg=300.0f,
        .min_femur_deg=0.0f, .max_femur_deg=300.0f,
        .min_tibia_deg=0.0f, .max_tibia_deg=300.0f,
    },
    /* Derecha delantera (RT). */
    {
        .name = "LEG_DERECHA DELANTERA",
        .id_coxa = 2, .id_femur = 4, .id_tibia = 6,
        .L = { .lc=52.0f, .lf=66.0f, .lt=135.0f, .xj=-120.0f, .yj=-60.0f, .eps=-(float)M_PI_4 },
        .off_coxa_deg=150.0f, .off_femur_deg=150.0f, .off_tibia_deg=150.0f,
        .trim_coxa_deg=0.0f,  .trim_femur_deg=0.0f,  .trim_tibia_deg=45.0f,
        .s_coxa=+1.0f, .s_femur=-1.0f, .s_tibia=-1.0f,
        .min_coxa_deg=0.0f, .max_coxa_deg=300.0f,
        .min_femur_deg=0.0f, .max_femur_deg=300.0f,
        .min_tibia_deg=0.0f, .max_tibia_deg=300.0f,
    },
};
