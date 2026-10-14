"""Nodo ROS 2 que conecta el panel web con el resto del hexápodo."""

import math
import threading
import time

import tf_transformations
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .payloads import (
    empty_odom_payload,
    empty_posture_axes,
    empty_servo_health_payload,
)
from .servo_registers import ServoRegisterManager


class WebPanelNode(Node):
    """Nodo ROS2 que publica un panel web simple para el hexapodo."""

    def __init__(self):
        super().__init__(
            'web_panel',
            automatically_declare_parameters_from_overrides=True,
        )

        self.web_port = self._int_param('web_port')
        self.posture_lock = threading.Lock()
        self.latest_odom = empty_odom_payload()
        self.latest_servo_health = empty_servo_health_payload()
        self.latest_posture_target = empty_posture_axes(None)
        self.latest_posture_current = empty_posture_axes(None)
        self.latest_smoothing_time = None
        self.latest_mode = None
        self.latest_gait_mode = None
        self.latest_gait_period_s = None
        self.latest_step_height_mm = None

        self.robot_command_pub = self.create_publisher(String, 'hexapod/robot_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/hexapod/cmd_vel', 1)
        self.control_mode = 'manual'
        self.last_avoider_time = None
        self.control_mode_lock = threading.Lock()
        self.avoider_velocity_sub = self.create_subscription(
            Twist, '/obstacle_avoider/cmd_vel', self.avoider_velocity_cb, 1,
        )
        self.servo_registers = ServoRegisterManager(self.send_robot_command)
        self.odom_sub = self.create_subscription(
            Odometry,
            'hexapod/odom_model',
            self.odom_cb,
            10,
        )
        self.body_pose_sub = self.create_subscription(
            Pose, 'hexapod/body_pose', self.body_pose_cb, 10,
        )
        self.robot_command_sub = self.create_subscription(
            String, 'hexapod/robot_command', self.robot_command_cb, 10,
        )
        self.servo_health_sub = self.create_subscription(
            String,
            'hexapod/servo_health',
            self.servo_health_cb,
            50,
        )
        self.servo_register_sub = self.create_subscription(
            String,
            'hexapod/servo_register',
            self.servo_register_cb,
            10,
        )
        self.ui_config = self._build_ui_config()
        self.get_logger().info('Panel web listo')

    def _int_param(self, name):
        return int(self.get_parameter(name).value)

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    def _axis_config(self, prefix):
        return {
            'min': self._int_param(f'{prefix}.min'),
            'max': self._int_param(f'{prefix}.max'),
            'step': self._int_param(f'{prefix}.step'),
        }

    def _build_ui_config(self):
        return {
            'motion': {
                'cmdVelRateHz': self._float_param('motion.cmd_vel_rate_hz'),
                'maxStepXMm': self._float_param(
                    'motion.max_step_x_mm'
                ),
                'maxStepYMm': self._float_param(
                    'motion.max_step_y_mm'
                ),
                'maxYawStepRad': self._float_param(
                    'motion.max_yaw_step_rad'
                ),
            },
            'posture': {
                'rollDeg': self._axis_config('posture.roll_deg'),
                'pitchDeg': self._axis_config('posture.pitch_deg'),
                'yawDeg': self._axis_config('posture.yaw_deg'),
                'xMm': self._axis_config('posture.x_mm'),
                'yMm': self._axis_config('posture.y_mm'),
                'zMm': self._axis_config('posture.z_mm'),
            },
        }

    def odom_cb(self, msg):
        self.latest_odom['x'] = round(msg.pose.pose.position.x * 1000, 2)
        self.latest_odom['y'] = round(msg.pose.pose.position.y * 1000, 2)

        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (
                orientation.w * orientation.z +
                orientation.x * orientation.y
            ),
            1 - 2 * (
                orientation.y * orientation.y +
                orientation.z * orientation.z
            ),
        )
        self.latest_odom['yaw'] = round(yaw, 4)

    def robot_command_cb(self, msg):
        self._update_robot_state_from_command(msg.data)

    def _update_robot_state_from_command(self, command_text):
        """Conserva el último estado solicitado por los comandos del robot."""
        parts = command_text.split(',')
        try:
            if len(parts) == 2 and parts[0] == 'MOD':
                mode = int(parts[1])
                if mode not in (0, 1, 2, 3):
                    return
                self.latest_mode = mode
                if mode == 0:
                    self.set_control_mode('manual')
                if mode != 0:
                    self.latest_gait_mode = mode
            elif len(parts) == 3 and parts[0] == 'GAT':
                period = float(parts[1])
                height = float(parts[2])
                if math.isfinite(period) and math.isfinite(height):
                    self.latest_gait_period_s = period
                    self.latest_step_height_mm = height
            elif len(parts) == 7 and parts[0] == 'POS':
                target = {
                    'roll': round(math.degrees(float(parts[1])), 1),
                    'pitch': round(math.degrees(float(parts[2])), 1),
                    'yaw': round(math.degrees(float(parts[3])), 1),
                    'x': round(float(parts[4]), 1),
                    'y': round(float(parts[5]), 1),
                    'z': round(float(parts[6]), 1),
                }
                with self.posture_lock:
                    self.latest_posture_target = target
            elif len(parts) == 2 and parts[0] == 'SMO':
                smoothing_time = float(parts[1])
                if math.isfinite(smoothing_time) and smoothing_time > 0.0:
                    with self.posture_lock:
                        self.latest_smoothing_time = smoothing_time
        except (IndexError, ValueError):
            return

    def body_pose_cb(self, msg):
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        current = {
            'roll': round(math.degrees(roll), 1),
            'pitch': round(math.degrees(pitch), 1),
            'yaw': round(math.degrees(yaw), 1),
            'x': round(msg.position.x * 1000.0, 1),
            'y': round(msg.position.y * 1000.0, 1),
            'z': round(msg.position.z * 1000.0, 1),
        }

        with self.posture_lock:
            self.latest_posture_current = current

    def servo_health_cb(self, msg):
        parts = msg.data.split(',')
        if len(parts) != 7 or parts[0] != 'AXH':
            return

        try:
            servo_id = int(parts[1])
            valid = parts[2] == '1'
            voltage_raw = int(parts[3])
            temperature_c = int(parts[4])
            load_raw = int(parts[5])
            error_raw = int(parts[6])
        except ValueError:
            return

        if servo_id < 1 or servo_id > 18:
            return

        load_mag = load_raw & 0x03FF
        load_dir = '-' if (load_raw & 0x0400) else '+'
        servo_payload = {
            'id': servo_id,
            'valid': valid,
            'voltage_v': round(voltage_raw / 10.0, 1) if valid else None,
            'temperature_c': temperature_c if valid else None,
            'load_raw': load_raw if valid else None,
            'load_pct': round((load_mag * 100.0) / 1023.0, 1) if valid else None,
            'load_dir': load_dir if valid else None,
            'error_raw': error_raw if valid else None,
        }

        self.latest_servo_health['servos'][servo_id - 1] = servo_payload

    def servo_register_cb(self, msg):
        """Entrega al gestor una respuesta AXR o AXW recibida por ROS 2."""
        self.servo_registers.process_response(msg.data)

    def send_robot_command(self, command_text):
        self._update_robot_state_from_command(command_text)
        msg = String()
        msg.data = command_text
        self.robot_command_pub.publish(msg)
        self.get_logger().info(f'CMD: {command_text}')

    def send_velocity(self, linear_x, linear_y, angular_z):
        with self.control_mode_lock:
            if self.control_mode != 'manual':
                return False
            msg = Twist()
            msg.linear.x = linear_x
            msg.linear.y = linear_y
            msg.angular.z = angular_z
            self.cmd_vel_pub.publish(msg)
            return True

    def avoider_velocity_cb(self, msg):
        with self.control_mode_lock:
            values = (
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z,
            )
            if not all(math.isfinite(value) for value in values):
                self.last_avoider_time = None
                if self.control_mode == 'autonomous':
                    self.cmd_vel_pub.publish(Twist())
                return
            self.last_avoider_time = time.monotonic()
            if self.control_mode == 'autonomous':
                self.cmd_vel_pub.publish(msg)

    def get_control_mode(self):
        with self.control_mode_lock:
            return {'mode': self.control_mode, 'available': True}

    def set_control_mode(self, mode):
        with self.control_mode_lock:
            if mode not in ('manual', 'autonomous'):
                return {'ok': False, 'error': 'Modo de control no valido'}
            if mode == 'autonomous' and (
                self.last_avoider_time is None
                or time.monotonic() - self.last_avoider_time >= 0.5
            ):
                return {'ok': False, 'error': 'No hay propuestas recientes del evitador'}
            self.control_mode = mode
            self.cmd_vel_pub.publish(Twist())
            return {'ok': True, 'mode': mode, 'available': True}

    def get_servo_health_payload(self):
        return {
            'servos': [dict(item) for item in self.latest_servo_health['servos']]
        }

    def get_posture_payload(self):
        with self.posture_lock:
            return {
                'target': dict(self.latest_posture_target),
                'current': dict(self.latest_posture_current),
                'smoothing_time': self.latest_smoothing_time,
            }

    def get_robot_state_payload(self):
        """Devuelve la configuración que debe conservarse al recargar el panel."""
        return {
            'mode': self.latest_mode,
            'gait_mode': self.latest_gait_mode,
            'gait_period_s': self.latest_gait_period_s,
            'step_height_mm': self.latest_step_height_mm,
            'smoothing_time': self.latest_smoothing_time,
        }
