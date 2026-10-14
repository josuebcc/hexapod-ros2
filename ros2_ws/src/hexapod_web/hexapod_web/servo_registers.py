"""Catálogo y operaciones de registros del servo Dynamixel AX-12A."""

import threading
import time


def register_definition(key, name, address, size, area, access='RW',
                        value_format='raw', expected=None,
                        write_enabled=None):
    """Construye una entrada del catálogo de registros AX-12A."""
    if write_enabled is None:
        write_enabled = access == 'RW'

    return {
        'key': key,
        'name': name,
        'address': address,
        'size': size,
        'area': area,
        'access': access,
        'format': value_format,
        'expected': expected,
        'write_enabled': write_enabled,
    }


SERVO_REGISTER_CATALOG = (
    register_definition('model_number', 'Model Number', 0, 2, 'EEPROM', 'R'),
    register_definition('firmware', 'Firmware Version', 2, 1, 'EEPROM', 'R'),
    register_definition('id', 'ID', 3, 1, 'EEPROM', write_enabled=False),
    register_definition('baud_rate', 'Baud Rate', 4, 1, 'EEPROM', write_enabled=False),
    register_definition(
        'return_delay', 'Return Delay Time', 5, 1, 'EEPROM',
        value_format='return_delay', expected=250,
    ),
    register_definition(
        'cw_angle_limit', 'CW Angle Limit', 6, 2, 'EEPROM',
        value_format='position',
    ),
    register_definition(
        'ccw_angle_limit', 'CCW Angle Limit', 8, 2, 'EEPROM',
        value_format='position',
    ),
    register_definition(
        'temperature_limit', 'Temperature Limit', 11, 1, 'EEPROM',
        value_format='temperature',
    ),
    register_definition(
        'min_voltage_limit', 'Min Voltage Limit', 12, 1, 'EEPROM',
        value_format='voltage',
    ),
    register_definition(
        'max_voltage_limit', 'Max Voltage Limit', 13, 1, 'EEPROM',
        value_format='voltage',
    ),
    register_definition(
        'max_torque', 'Max Torque', 14, 2, 'EEPROM', value_format='torque',
    ),
    register_definition(
        'status_return_level', 'Status Return Level', 16, 1, 'EEPROM',
        write_enabled=False,
    ),
    register_definition(
        'alarm_led', 'Alarm LED', 17, 1, 'EEPROM',
        value_format='bitmask', expected=0x25,
    ),
    register_definition(
        'shutdown', 'Shutdown', 18, 1, 'EEPROM',
        value_format='bitmask', expected=0x04,
    ),
    register_definition(
        'torque_enable', 'Torque Enable', 24, 1, 'RAM', value_format='boolean',
    ),
    register_definition('led', 'LED', 25, 1, 'RAM', value_format='boolean'),
    register_definition('cw_margin', 'CW Compliance Margin', 26, 1, 'RAM', expected=4),
    register_definition('ccw_margin', 'CCW Compliance Margin', 27, 1, 'RAM', expected=4),
    register_definition('cw_slope', 'CW Compliance Slope', 28, 1, 'RAM', expected=64),
    register_definition('ccw_slope', 'CCW Compliance Slope', 29, 1, 'RAM', expected=64),
    register_definition(
        'goal_position', 'Goal Position', 30, 2, 'RAM', value_format='position',
    ),
    register_definition(
        'moving_speed', 'Moving Speed', 32, 2, 'RAM', value_format='speed',
    ),
    register_definition(
        'torque_limit', 'Torque Limit', 34, 2, 'RAM', value_format='torque',
    ),
    register_definition(
        'present_position', 'Present Position', 36, 2, 'RAM', 'R',
        value_format='position',
    ),
    register_definition(
        'present_speed', 'Present Speed', 38, 2, 'RAM', 'R',
        value_format='speed',
    ),
    register_definition(
        'present_load', 'Present Load', 40, 2, 'RAM', 'R', value_format='load',
    ),
    register_definition(
        'present_voltage', 'Present Voltage', 42, 1, 'RAM', 'R',
        value_format='voltage',
    ),
    register_definition(
        'present_temperature', 'Present Temperature', 43, 1, 'RAM', 'R',
        value_format='temperature',
    ),
    register_definition(
        'registered_instruction', 'Registered Instruction', 44, 1, 'RAM', 'R',
        value_format='boolean',
    ),
    register_definition('moving', 'Moving', 46, 1, 'RAM', 'R', value_format='boolean'),
    register_definition(
        'lock', 'Lock', 47, 1, 'RAM', value_format='boolean', write_enabled=False,
    ),
    register_definition('punch', 'Punch', 48, 2, 'RAM'),
)

SERVO_REGISTER_BY_KEY = {item['key']: item for item in SERVO_REGISTER_CATALOG}
SERVO_REGISTER_BY_LOCATION = {
    (item['address'], item['size']): item for item in SERVO_REGISTER_CATALOG
}


class ServoRegisterManager:
    """Gestiona una operación de lectura o escritura AX-12A cada vez."""

    def __init__(self, send_command):
        self._send_command = send_command
        self._lock = threading.Lock()
        self._results = {}

    @staticmethod
    def _error(message):
        return {'ok': False, 'error': message}

    @staticmethod
    def _valid_register(servo_id, register_key):
        register = SERVO_REGISTER_BY_KEY.get(register_key)
        if 1 <= servo_id <= 18 and register is not None:
            return register
        return None

    def _begin(self, operation, servo_id, register_key, value=None):
        now = time.time()
        payload = {
            'ok': True,
            'pending': True,
            'operation': operation,
            'servo_id': servo_id,
            'register': register_key,
            'valid': False,
            'value': value,
            'error_raw': None,
            'updated_at': now,
        }

        with self._lock:
            busy = any(
                result.get('pending')
                and now - result.get('updated_at', 0.0) < 3.0
                for result in self._results.values()
            )
            if busy:
                return self._error('Ya hay una operación de registro en curso')
            self._results[(servo_id, register_key)] = payload

        return dict(payload)

    def process_response(self, command_text):
        """Guarda una respuesta AXR o AXW recibida desde el ESP32."""
        parts = command_text.split(',')
        if len(parts) != 7 or parts[0] not in ('AXR', 'AXW'):
            return

        try:
            servo_id = int(parts[1])
            address = int(parts[2])
            size = int(parts[3])
            valid = parts[4] == '1'
            value = int(parts[5])
            error_raw = int(parts[6])
        except ValueError:
            return

        register = SERVO_REGISTER_BY_LOCATION.get((address, size))
        if register is None or not 1 <= servo_id <= 18:
            return

        operation = 'write' if parts[0] == 'AXW' else 'read'
        payload = {
            'ok': True,
            'pending': False,
            'operation': operation,
            'servo_id': servo_id,
            'register': register['key'],
            'valid': valid,
            'value': value if valid or operation == 'write' else None,
            'error_raw': error_raw if valid or operation == 'write' else None,
            'updated_at': time.time(),
        }

        key = (servo_id, register['key'])
        with self._lock:
            current = self._results.get(key)
            if (current is not None and current.get('pending')
                    and current.get('operation') != operation):
                return
            self._results[key] = payload

    def request_read(self, servo_id, register_key):
        """Solicita la lectura de un registro válido."""
        register = self._valid_register(servo_id, register_key)
        if register is None:
            return self._error('Servo o registro no válido')

        payload = self._begin('read', servo_id, register_key)
        if payload['ok']:
            self._send_command(
                f"REG,{servo_id},{register['address']},{register['size']}"
            )
        return payload

    def request_write(self, servo_id, register_key, value):
        """Solicita la escritura de un registro habilitado."""
        register = self._valid_register(servo_id, register_key)
        if register is None or not register['write_enabled']:
            return self._error('Servo o registro de escritura no válido')

        max_value = 0xFF if register['size'] == 1 else 0xFFFF
        if not 0 <= value <= max_value:
            return self._error(f'El valor debe estar entre 0 y {max_value}')

        payload = self._begin('write', servo_id, register_key, value)
        if payload['ok']:
            self._send_command(
                f"WREG,{servo_id},{register['address']},"
                f"{register['size']},{value}"
            )
        return payload

    def get_result(self, servo_id, register_key):
        """Devuelve el último resultado conocido de un registro."""
        if self._valid_register(servo_id, register_key) is None:
            return self._error('Servo o registro no válido')

        with self._lock:
            payload = self._results.get((servo_id, register_key))

        if payload is not None:
            return dict(payload)
        return {
            'ok': True,
            'pending': False,
            'operation': None,
            'servo_id': servo_id,
            'register': register_key,
            'valid': False,
            'value': None,
            'error_raw': None,
            'updated_at': None,
        }
