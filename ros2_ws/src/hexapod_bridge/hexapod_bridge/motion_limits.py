import math
from dataclasses import dataclass
from enum import IntEnum

# Modos de marcha definidos por el firmware.

class GaitMode(IntEnum):
    """Modos de locomoción definidos por el protocolo de la ESP32."""

    STOP = 0
    TRIPOD = 1
    RIPPLE = 2
    AMBLE = 3


# Proporción del ciclo durante la que cada pata permanece apoyada.
# STOP no necesita valores propios porque conserva el último patrón activo.
DUTY_CYCLES_BY_MODE = {
    GaitMode.TRIPOD: 1.0 / 2.0,
    GaitMode.RIPPLE: 5.0 / 6.0,
    GaitMode.AMBLE: 2.0 / 3.0,
}

# Periodo mínimo experimental, en segundos, para mantener una marcha estable.
MIN_GAIT_PERIODS_BY_MODE = {
    GaitMode.TRIPOD: 1.0,
    GaitMode.RIPPLE: 3.0,
    GaitMode.AMBLE: 1.5,
}

# Intervalo de altura del paso, en milímetros, común a todas las marchas.
MIN_STEP_HEIGHT_MM = 10.0
MAX_STEP_HEIGHT_MM = 60.0

# Una misma estructura representa tanto límites como velocidades ya limitadas.

@dataclass(frozen=True)
class PlanarVelocity:
    """Velocidad lineal en X/Y y velocidad angular alrededor de Z."""

    vx_mm_s: float
    vy_mm_s: float
    yaw_rad_s: float

def calculate_motion_limits(gait_mode, period_s, max_step_x_mm, max_step_y_mm, max_yaw_step_rad):
    """Calcula las velocidades máximas permitidas por la marcha activa."""
    # Duración del apoyo = periodo completo × proporción de apoyo.
    duty_cycle = DUTY_CYCLES_BY_MODE[gait_mode]
    stance_duration = period_s * duty_cycle
    # Velocidad máxima = recorrido máximo / duración del apoyo.
    return PlanarVelocity(max_step_x_mm / stance_duration, max_step_y_mm / stance_duration, max_yaw_step_rad / stance_duration)


def limit_velocity(vx_mm_s, vy_mm_s, yaw_rad_s, limits):
    """Escala conjuntamente un comando que supera los límites de marcha."""
    if not all(math.isfinite(value) for value in (vx_mm_s, vy_mm_s, yaw_rad_s)):
        raise ValueError('La velocidad solicitada debe contener valores finitos')

    # Se suman las tres demandas normalizadas; 1.0 representa el 100 %.
    total_demand = abs(vx_mm_s) / limits.vx_mm_s + abs(vy_mm_s) / limits.vy_mm_s + abs(yaw_rad_s) / limits.yaw_rad_s
    # Si se supera el 100 %, se escalan las tres componentes por igual.
    scale = 1.0 / total_demand if total_demand > 1.0 else 1.0
    return PlanarVelocity(vx_mm_s * scale, vy_mm_s * scale, yaw_rad_s * scale)


def limit_gait_parameters(period_s, step_height_mm, gait_mode):
    """Aplica los límites de periodo y altura de la marcha seleccionada."""
    # El periodo mínimo depende del patrón; la altura es común a todos.
    limited_period = max(period_s, MIN_GAIT_PERIODS_BY_MODE[gait_mode])
    limited_height = min(max(step_height_mm, MIN_STEP_HEIGHT_MM), MAX_STEP_HEIGHT_MM)
    return limited_period, limited_height
