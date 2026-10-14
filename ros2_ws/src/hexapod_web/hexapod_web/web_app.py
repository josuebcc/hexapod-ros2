"""Aplicación Flask y rutas HTTP del panel."""

import math
import os

from flask import Flask, jsonify, render_template, request

from .servo_registers import SERVO_REGISTER_CATALOG

TEMPLATE_DIR = os.path.join(os.path.dirname(__file__), 'templates')


def create_app(node):
    """Crea la aplicación HTTP conectada al nodo ROS 2 indicado."""
    app = Flask(__name__, template_folder=TEMPLATE_DIR)

    @app.get('/')
    def index():
        return render_template('index.html')

    @app.get('/servos')
    def servos():
        return render_template('servos.html')

    @app.post('/robot_command')
    def robot_command():
        data = request.get_json(silent=True) or {}
        node.send_robot_command(data.get('cmd', ''))
        return jsonify({'ok': True})

    @app.post('/cmd_vel')
    def cmd_vel():
        data = request.get_json(silent=True) or {}
        try:
            velocity = (
                float(data.get('linear_x', 0.0)),
                float(data.get('linear_y', 0.0)),
                float(data.get('angular_z', 0.0)),
            )
        except (TypeError, ValueError):
            return jsonify({
                'ok': False,
                'error': 'La velocidad debe contener valores numéricos',
            }), 400

        if not all(math.isfinite(value) for value in velocity):
            return jsonify({
                'ok': False,
                'error': 'La velocidad debe contener valores finitos',
            }), 400

        if not node.send_velocity(*velocity):
            return jsonify({'ok': False, 'error': 'Control manual inactivo'}), 409
        return jsonify({'ok': True})

    @app.get('/odom')
    def odom():
        return jsonify(node.latest_odom)

    @app.get('/control_mode')
    def control_mode():
        return jsonify(node.get_control_mode())

    @app.post('/control_mode')
    def set_control_mode():
        data = request.get_json(silent=True)
        if not isinstance(data, dict) or data.get('mode') not in ('manual', 'autonomous'):
            return jsonify({'ok': False, 'error': 'Modo de control no valido'}), 400
        payload = node.set_control_mode(data['mode'])
        return jsonify(payload), 200 if payload['ok'] else 503

    @app.get('/config')
    def config():
        return jsonify(node.ui_config)

    @app.get('/posture')
    def posture():
        return jsonify(node.get_posture_payload())

    @app.get('/robot_state')
    def robot_state():
        return jsonify(node.get_robot_state_payload())

    @app.get('/servo_health')
    def servo_health():
        return jsonify(node.get_servo_health_payload())

    @app.get('/servo_registers')
    def servo_registers():
        return jsonify({
            'registers': [dict(item) for item in SERVO_REGISTER_CATALOG],
        })

    @app.route('/servo_register', methods=['GET', 'POST', 'PUT'])
    def servo_register():
        data = (
            request.get_json(silent=True) or {}
            if request.method in ('POST', 'PUT')
            else request.args
        )

        try:
            servo_id = int(data.get('servo_id', 0))
        except (TypeError, ValueError):
            servo_id = 0
        register_key = str(data.get('register', ''))

        if request.method == 'POST':
            payload = node.servo_registers.request_read(servo_id, register_key)
            return jsonify(payload), 202 if payload.get('ok') else 400

        if request.method == 'PUT':
            try:
                value = int(data.get('value'))
            except (TypeError, ValueError):
                return jsonify({
                    'ok': False,
                    'error': 'Valor de escritura no válido',
                }), 400

            payload = node.servo_registers.request_write(
                servo_id,
                register_key,
                value,
            )
            return jsonify(payload), 202 if payload.get('ok') else 400

        payload = node.servo_registers.get_result(servo_id, register_key)
        return jsonify(payload), 200 if payload.get('ok') else 400

    return app
