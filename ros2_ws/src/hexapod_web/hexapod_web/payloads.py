"""Estados iniciales enviados por la API del panel."""


def empty_odom_payload():
    return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}


def empty_servo_health_payload():
    return {
        'servos': [
            {
                'id': servo_id,
                'valid': False,
                'voltage_v': None,
                'temperature_c': None,
                'load_raw': None,
                'load_pct': None,
                'load_dir': None,
                'error_raw': None,
            }
            for servo_id in range(1, 19)
        ],
    }


def empty_posture_axes(default_value):
    return {
        'roll': default_value,
        'pitch': default_value,
        'yaw': default_value,
        'x': default_value,
        'y': default_value,
        'z': default_value,
    }
