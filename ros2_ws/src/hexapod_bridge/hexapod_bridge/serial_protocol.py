import math
from dataclasses import dataclass

# Datos recibidos desde el ESP32.


@dataclass(frozen=True)
class OdometryTelemetry:
    """ODO: odometría teórica y postura actual del cuerpo."""

    x_mm: float
    y_mm: float
    yaw_rad: float
    roll_rad: float
    pitch_rad: float
    body_yaw_rad: float
    body_x_mm: float
    body_y_mm: float
    body_z_mm: float


@dataclass(frozen=True)
class ServoHealthTelemetry:
    """AXH: diagnóstico periódico de un actuador."""

    servo_id: int
    valid: int
    voltage_raw: int
    temperature_c: int
    load_raw: int
    error_raw: int


@dataclass(frozen=True)
class ServoRegisterTelemetry:
    """AXR/AXW: resultado de acceso a un registro AX-12A."""

    operation: str
    servo_id: int
    address: int
    size: int
    valid: int
    value: int
    error_raw: int

# Comandos enviados hacia el ESP32.
# MOD selecciona la marcha; GAT configura el paso; SMO ajusta la transición;
# CMD transporta velocidades lineales en mm/s y giro en rad/s.


def format_mode_command(mode):
    return f'MOD,{int(mode)}'


def format_gait_command(period_s, step_height_mm):
    return f'GAT,{period_s:g},{step_height_mm:g}'


def format_smoothing_command(time_s):
    return f'SMO,{time_s:g}'


def format_velocity_command(vx_mm_s, vy_mm_s, yaw_rad_s):
    return f'CMD,{vx_mm_s:.1f},{vy_mm_s:.1f},{yaw_rad_s:.3f}'

# Interpretación de comandos y telemetría recibida.


def parse_gait_command(text):
    """Extrae periodo y altura de un comando GAT válido."""
    parts = text.split(',')
    if len(parts) != 3 or parts[0] != 'GAT':
        return None
    return _parse_floats(parts[1:])


def parse_telemetry_line(text):
    """Convierte una línea de telemetría reconocida en datos estructurados."""
    parts = text.split(',')
    prefix = parts[0]
    if prefix == 'ODO' and len(parts) == 10:
        values = _parse_floats(parts[1:])
        return OdometryTelemetry(*values) if values is not None else None
    if prefix == 'AXH' and len(parts) == 7:
        values = _parse_ints(parts[1:])
        return ServoHealthTelemetry(*values) if values is not None else None
    if prefix in ('AXR', 'AXW') and len(parts) == 7:
        values = _parse_ints(parts[1:])
        return ServoRegisterTelemetry(prefix, *values) if values is not None else None
    return None


def _parse_floats(values):
    """Convierte una secuencia a decimales finitos."""
    try:
        parsed = tuple(float(value) for value in values)
    except ValueError:
        return None
    return parsed if all(math.isfinite(value) for value in parsed) else None


def _parse_ints(values):
    """Convierte una secuencia a números enteros."""
    try:
        return tuple(int(value) for value in values)
    except ValueError:
        return None
