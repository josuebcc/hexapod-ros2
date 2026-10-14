/**
 * @file body_kinematics.cpp
 * @brief Implementación de la cinemática y la odometría del cuerpo.
 */

#include "body_kinematics.h"

/* --------------------------------------------------------------------------
 * Inicialización
 * -------------------------------------------------------------------------- */

void body_kinematics_init(body_kinematics_t *bk, float initial_z_mm)
{
    // La trayectoria parte del origen del mundo y conserva la altura base.
    bk->T_tr = rigid_transform_from_rpy_xyz(
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, initial_z_mm);

    // No existe ningún desplazamiento local de postura al inicializar.
    bk->T_body = rigid_transform_identity();
}

/* --------------------------------------------------------------------------
 * Actualización del estado
 * -------------------------------------------------------------------------- */

void body_kinematics_set_posture_rpy_xyz(body_kinematics_t *bk,float roll,float pitch,float yaw,float x,float y,float z)
{
    bk->T_body = rigid_transform_from_rpy_xyz(roll, pitch, yaw, x, y, z);
}

void body_kinematics_integrate_gait_delta(body_kinematics_t *bk, float dx_body, float dy_body, float dyaw)
{
    // Incremento plano expresado en el frame de trayectoria actual.
    se3 gait_delta;
    gait_delta.rotation = rotation_matrix_z(dyaw);
    gait_delta.translation = vector3(dx_body, dy_body, 0.0f);

    // Acumulación local: T_tr,new = T_tr,old * gait_delta.
    bk->T_tr = rigid_transform_compose(&bk->T_tr, &gait_delta);
}

/* --------------------------------------------------------------------------
 * Pose y transformación de puntos
 * -------------------------------------------------------------------------- */

se3 body_kinematics_get_world_from_body(const body_kinematics_t *bk)
{
    return rigid_transform_compose(&bk->T_tr, &bk->T_body);
}

vec3 body_kinematics_point_world_to_body(const body_kinematics_t *bk, vec3 p_world)
{
    se3 T_gb = body_kinematics_get_world_from_body(bk);
    se3 T_bg = rigid_transform_inverse(&T_gb);
    return rigid_transform_apply_to_point(&T_bg, p_world);
}

/* --------------------------------------------------------------------------
 * Movimiento relativo
 * -------------------------------------------------------------------------- */

se3 body_kinematics_compute_qbody(const body_kinematics_t *bk_prev, const body_kinematics_t *bk_curr)
{
    se3 T_gb_prev = body_kinematics_get_world_from_body(bk_prev);
    se3 T_gb_curr = body_kinematics_get_world_from_body(bk_curr);
    se3 T_bg_prev = rigid_transform_inverse(&T_gb_prev);
    return rigid_transform_compose(&T_bg_prev, &T_gb_curr);
}

/* --------------------------------------------------------------------------
 * Odometría
 * -------------------------------------------------------------------------- */

void body_kinematics_get_odometry(const body_kinematics_t *bk, float *x, float *y, float *yaw)
{
    if (x) {
        *x = bk->T_tr.translation.x;
    }
    if (y) {
        *y = bk->T_tr.translation.y;
    }
    if (yaw) {
        // Para una rotación alrededor de Z: yaw = atan2(R21, R11).
        *yaw = atan2f(bk->T_tr.rotation.m[1][0],
                      bk->T_tr.rotation.m[0][0]);
    }
}

void body_kinematics_reset_odometry(body_kinematics_t *bk)
{
    float base_height_mm = bk->T_tr.translation.z;
    bk->T_tr = rigid_transform_from_rpy_xyz(
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, base_height_mm);
}
