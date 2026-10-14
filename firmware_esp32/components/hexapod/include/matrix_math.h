/**
 * @file matrix_math.h
 * @brief Operaciones con vectores, matrices y transformaciones rígidas.
 */

#pragma once

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup group_matrix_math matrix_math
 * @brief Álgebra tridimensional utilizada por la locomoción.
 *
 * El módulo reúne las operaciones matemáticas básicas que comparten la
 * cinemática del cuerpo, la cinemática de las patas y el generador de marcha.
 *
 * Se adoptan las siguientes convenciones:
 * - Los vectores se interpretan como vectores columna.
 * - Las matrices se almacenan como m[fila][columna].
 *
 * @par Código fuente
 * Declaración e implementación en matrix_math.h.
 * @{
 */

/* --------------------------------------------------------------------------
 * Tipos públicos
 * -------------------------------------------------------------------------- */

/** @name Tipos públicos
 *  @{ */

/** @brief Vector tridimensional. */
typedef struct {
    float x;  /**< Componente X. */
    float y;  /**< Componente Y. */
    float z;  /**< Componente Z. */
} vec3;

/**
 * @brief Matriz de 3 por 3.
 */
typedef struct {
    float m[3][3];  /**< Elementos de la matriz. */
} mat3;

/**
 * @brief Transformación rígida tridimensional que representa una rotación y una traslación
 */
typedef struct {
    mat3 rotation;     /**< Matriz de rotación R. */
    vec3 translation;  /**< Vector de traslación t. */
} se3;

/** @} */

/* --------------------------------------------------------------------------
 * Operaciones con escalares
 * -------------------------------------------------------------------------- */

/** @name Operaciones con escalares
 *  @{ */

/**
 * @brief Limita un escalar a un intervalo cerrado.
 *
 * @param[in] value Valor que se limitará.
 * @param[in] min_value Límite inferior.
 * @param[in] max_value Límite superior.
 * @return Valor limitado al intervalo [min_value, max_value].
 */
static inline float scalar_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * @brief Calcula el cuadrado de un escalar.
 *
 * @param[in] value Valor de entrada.
 * @return Cuadrado del valor.
 */
static inline float scalar_square(float value)
{
    return value * value;
}

/** @} */

/* --------------------------------------------------------------------------
 * Operaciones con vectores
 * -------------------------------------------------------------------------- */

/** @name Operaciones con vectores
 *  @{ */

/**
 * @brief Construye un vector tridimensional.
 *
 * @param[in] x Componente X.
 * @param[in] y Componente Y.
 * @param[in] z Componente Z.
 * @return Vector formado por las tres componentes.
 */
static inline vec3 vector3(float x, float y, float z)
{
    return (vec3){x, y, z};
}

/**
 * @brief Suma dos vectores
 *
 * @param[in] a Primer vector.
 * @param[in] b Segundo vector.
 * @return Vector a+b.
 */
static inline vec3 vector3_add(vec3 a, vec3 b)
{
    return vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/**
 * @brief Resta dos vectores componente a componente.
 *
 * @param[in] a Vector minuendo.
 * @param[in] b Vector sustraendo.
 * @return Vector a-b.
 */
static inline vec3 vector3_sub(vec3 a, vec3 b)
{
    return vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/** @} */

/* --------------------------------------------------------------------------
 * Operaciones con matrices
 * -------------------------------------------------------------------------- */

/** @name Operaciones con matrices
 *  @{ */

/**
 * @brief Devuelve la matriz identidad de 3 por 3.
 *
 * @return Matriz identidad.
 */
static inline mat3 matrix3_identity(void)
{
    mat3 identity = { .m = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} };
    return identity;
}

/**
 * @brief Multiplica dos matrices de 3 por 3.
 *
 * Calcula result=A*B. El resultado se construye en una variable temporal
 *
 * @param[in] A Matriz izquierda.
 * @param[in] B Matriz derecha.
 * @param[out] result Matriz donde se almacena el producto.
 */
static inline void matrix3_multiply(const mat3 *A, const mat3 *B, mat3 *result)
{
    mat3 tmp;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tmp.m[i][j] =
                A->m[i][0] * B->m[0][j] +
                A->m[i][1] * B->m[1][j] +
                A->m[i][2] * B->m[2][j];
        }
    }

    *result = tmp;
}

/**
 * @brief Aplica una matriz de 3 por 3 a un vector.
 *
 * @param[in] R Matriz que se aplicará.
 * @param[in] v Vector de entrada.
 * @return Vector Rv.
 */
static inline vec3 matrix3_apply_to_vector(const mat3 *R, vec3 v)
{
    return vector3(
        R->m[0][0] * v.x + R->m[0][1] * v.y + R->m[0][2] * v.z,
        R->m[1][0] * v.x + R->m[1][1] * v.y + R->m[1][2] * v.z,
        R->m[2][0] * v.x + R->m[2][1] * v.y + R->m[2][2] * v.z
    );
}

/**
 * @brief Aplica la transpuesta de una matriz a un vector.
 *
 * Para una matriz de rotación, esta operación equivale a aplicar su inversa.
 *
 * @param[in] R Matriz cuya transpuesta se aplicará.
 * @param[in] v Vector de entrada.
 * @return Vector R^T * v.
 */
static inline vec3 matrix3_apply_transpose_to_vector(const mat3 *R, vec3 v)
{
    return vector3(
        R->m[0][0] * v.x + R->m[1][0] * v.y + R->m[2][0] * v.z,
        R->m[0][1] * v.x + R->m[1][1] * v.y + R->m[2][1] * v.z,
        R->m[0][2] * v.x + R->m[1][2] * v.y + R->m[2][2] * v.z
    );
}

/**
 * @brief Calcula la transpuesta de una matriz de 3 por 3.
 *
 * @param[in] R Matriz de entrada.
 * @return Matriz transpuesta R^T .
 */
static inline mat3 matrix3_transpose(const mat3 *R)
{
    mat3 transpose;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            transpose.m[i][j] = R->m[j][i];
        }
    }

    return transpose;
}

/** @} */

/* --------------------------------------------------------------------------
 * Matrices de rotación
 * -------------------------------------------------------------------------- */

/** @name Matrices de rotación
 *  @{ */

/**
 * @brief Construye una rotación alrededor del eje X.
 *
 * @param[in] roll Ángulo de giro en radianes.
 * @return Matriz de rotación R_x(roll).
 */
static inline mat3 rotation_matrix_x(float roll)
{
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    mat3 rotation = {
        .m = {{1, 0, 0}, {0, cos_roll, -sin_roll}, {0, sin_roll, cos_roll}}
    };
    return rotation;
}

/**
 * @brief Construye una rotación alrededor del eje Y.
 *
 * @param[in] pitch Ángulo de giro en radianes.
 * @return Matriz de rotación R_y(pitch).
 */
static inline mat3 rotation_matrix_y(float pitch)
{
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);
    mat3 rotation = {
        .m = {{cos_pitch, 0, sin_pitch}, {0, 1, 0}, {-sin_pitch, 0, cos_pitch}}
    };
    return rotation;
}

/**
 * @brief Construye una rotación alrededor del eje Z.
 *
 * @param[in] yaw Ángulo de giro en radianes.
 * @return Matriz de rotación R_z(yaw).
 */
static inline mat3 rotation_matrix_z(float yaw)
{
    float sin_yaw = sinf(yaw);
    float cos_yaw = cosf(yaw);
    mat3 rotation = {
        .m = {{cos_yaw, -sin_yaw, 0}, {sin_yaw, cos_yaw, 0}, {0, 0, 1}}
    };
    return rotation;
}

/**
 * @brief Construye una matriz de rotación a partir de roll, pitch y yaw.
 *
 * La composición utilizada es
 * R=R_z(yaw)R_y(pitch)R_x(roll).
 *
 * @param[in] roll Rotación alrededor de X, en radianes.
 * @param[in] pitch Rotación alrededor de Y, en radianes.
 * @param[in] yaw Rotación alrededor de Z, en radianes.
 * @return Matriz de rotación compuesta.
 */
static inline mat3 rotation_matrix_from_rpy(float roll,
                                            float pitch,
                                            float yaw)
{
    mat3 rotation_z = rotation_matrix_z(yaw);
    mat3 rotation_y = rotation_matrix_y(pitch);
    mat3 rotation_x = rotation_matrix_x(roll);
    mat3 rotation_zy;
    mat3 rotation;

    matrix3_multiply(&rotation_z, &rotation_y, &rotation_zy);
    matrix3_multiply(&rotation_zy, &rotation_x, &rotation);

    return rotation;
}

/** @} */

/* --------------------------------------------------------------------------
 * Transformaciones rígidas
 * -------------------------------------------------------------------------- */

/** @name Transformaciones rígidas
 *  @{ */

/**
 * @brief Devuelve la transformación rígida identidad.
 *
 * @return Transformación sin rotación ni traslación.
 */
static inline se3 rigid_transform_identity(void)
{
    se3 transform;
    transform.rotation = matrix3_identity();
    transform.translation = vector3(0.0f, 0.0f, 0.0f);
    return transform;
}

/**
 * @brief Calcula la inversa de una transformación rígida.
 *
 * @param[in] T Transformación que se invertirá.
 * @return Transformación inversa.
 */
static inline se3 rigid_transform_inverse(const se3 *T)
{
    se3 inverse;
    inverse.rotation = matrix3_transpose(&T->rotation);

    vec3 rotated_translation = matrix3_apply_to_vector(&inverse.rotation, T->translation);
    inverse.translation = vector3(-rotated_translation.x,-rotated_translation.y,-rotated_translation.z);

    return inverse;
}

/**
 * @brief Construye una transformación desde una orientación y una traslación.
 *
 * @param[in] roll Rotación alrededor de X, en radianes.
 * @param[in] pitch Rotación alrededor de Y, en radianes.
 * @param[in] yaw Rotación alrededor de Z, en radianes.
 * @param[in] x Traslación en X.
 * @param[in] y Traslación en Y.
 * @param[in] z Traslación en Z.
 * @return Transformación rígida resultante.
 */
static inline se3 rigid_transform_from_rpy_xyz(float roll,float pitch,float yaw,float x,float y,float z)
{
    se3 transform;
    transform.rotation = rotation_matrix_from_rpy(roll, pitch, yaw);
    transform.translation = vector3(x, y, z);
    return transform;
}

/**
 * @brief Compone dos transformaciones rígidas.
 *
 * @param[in] A Transformación que se aplica en segundo lugar.
 * @param[in] B Transformación que se aplica en primer lugar.
 * @return Transformación compuesta.
 */
static inline se3 rigid_transform_compose(const se3 *A,const se3 *B)
{
    se3 result;
    matrix3_multiply(&A->rotation, &B->rotation, &result.rotation);
    result.translation = vector3_add(matrix3_apply_to_vector(&A->rotation, B->translation),A->translation);
    return result;
}

/**
 * @brief Aplica una transformación rígida a un punto.
 *
 * @param[in] T Transformación que se aplicará.
 * @param[in] p Punto de entrada.
 * @return Punto transformado 
 */
static inline vec3 rigid_transform_apply_to_point(const se3 *T, vec3 p)
{
    return vector3_add(matrix3_apply_to_vector(&T->rotation, p),T->translation);
}

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
