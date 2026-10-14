import threading
import time

import rclpy
import serial
import tf_transformations
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from serial import SerialException
from std_msgs.msg import String

from .motion_limits import (
    GaitMode,
    calculate_motion_limits,
    limit_gait_parameters,
    limit_velocity,
)
from .serial_protocol import (
    OdometryTelemetry,
    ServoHealthTelemetry,
    ServoRegisterTelemetry,
    format_gait_command,
    format_mode_command,
    format_smoothing_command,
    format_velocity_command,
    parse_gait_command,
    parse_telemetry_line,
)

MIN_SMOOTHING_TIME_S = 0.5


class ESP32BridgeNode(Node):
    """Nodo ROS2 que traduce cmd_vel a comandos serie para la ESP32."""

    def __init__(self):
        super().__init__('esp32_bridge', automatically_declare_parameters_from_overrides=True)

        self._load_parameters()

        # Abrir el puerto serie.
        self.ser = self._open_serial()

        # Crear publicadores de odometría, postura y diagnóstico de servos.
        self.odom_pub = self.create_publisher(Odometry, 'hexapod/odom_model', 1)
        self.body_pose_pub = self.create_publisher(Pose, 'hexapod/body_pose', 10)
        self.servo_health_pub = self.create_publisher(String, 'hexapod/servo_health', 50)
        self.servo_register_pub = self.create_publisher(String, 'hexapod/servo_register', 10)

        # Crear suscriptores para velocidad y órdenes generales del robot.
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'hexapod/cmd_vel', self.cmd_vel_callback, 1,
        )
        self.robot_command_sub = self.create_subscription(
            String, 'hexapod/robot_command', self.robot_command_callback, 10,
        )

        # Detener el movimiento si deja de renovarse cmd_vel.
        self._last_cmd_vel_time = None
        self._cmd_vel_active = False
        self._cmd_vel_timeout_timer = self.create_timer(0.1, self._check_cmd_vel_timeout)

        # Iniciar el hilo de lectura continua del puerto serie.
        self._serial_stop_event = threading.Event()
        self.serial_thread = threading.Thread(
            target=self.read_serial_loop,
            daemon=True,
        )
        self.serial_thread.start()

        # Dar tiempo al ESP32 para arrancar antes de configurarlo.
        self._startup_timer = self.create_timer(2.0, self._run_startup_sequence)

    # Configuración inicial y conexión serie.

    def _load_parameters(self):
        """Lee y valida la configuración inicial del puente."""
        requested_mode = int(self.get_parameter('startup_mode').value)
        if requested_mode not in GaitMode:
            self.get_logger().warning(f'Modo inicial {requested_mode} no válido; se usará TRIPOD')
            requested_mode = GaitMode.TRIPOD
        self.startup_mode = GaitMode(requested_mode)
        self.current_gait_mode = (
            GaitMode.TRIPOD
            if self.startup_mode == GaitMode.STOP
            else self.startup_mode
        )

        requested_period = float(self.get_parameter('current_period').value)
        requested_height = float(self.get_parameter('step_height_mm').value)
        self.current_period, self.startup_step_height_mm = limit_gait_parameters(
            requested_period, requested_height, self.current_gait_mode,
        )

        requested_smoothing = float(self.get_parameter('smoothing_time').value)
        self.startup_smoothing_time = max(requested_smoothing, MIN_SMOOTHING_TIME_S)
        self.cmd_vel_timeout_s = float(self.get_parameter('cmd_vel_timeout_s').value)

        self.max_step_x_mm = float(self.get_parameter('max_step_x_mm').value)
        self.max_step_y_mm = float(self.get_parameter('max_step_y_mm').value)
        self.max_yaw_step_rad = float(self.get_parameter('max_yaw_step_rad').value)

    def _open_serial(self):
        """Abre el puerto serie configurado para comunicarse con el ESP32."""
        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)

        try:
            serial_handle = serial.Serial(port, baud, timeout=0.1)
        except (OSError, SerialException) as exc:
            self.get_logger().error(
                f'No se pudo abrir el puerto serie {port}: {exc}'
            )
            raise

        self.get_logger().info(f'Conectado a {port}')
        return serial_handle

    def _run_startup_sequence(self):
        """Envía una vez la configuración inicial del robot."""
        self._startup_timer.cancel()

        self.get_logger().info('Despertando al hexapodo...')

        commands = (
            format_mode_command(self.startup_mode),
            format_gait_command(self.current_period, self.startup_step_height_mm),
            format_smoothing_command(self.startup_smoothing_time),
        )

        for command in commands:
            self.send_robot_command(command)

        self.get_logger().info(
            'Robot configurado. Esperando comandos ROS y serial...'
        )

    # Envío de comandos: ROS 2 -> ESP32.

    def cmd_vel_callback(self, msg):
        """Convierte una velocidad ROS en un comando de movimiento limitado."""
        vx_req = msg.linear.x * 1000.0
        vy_req = msg.linear.y * 1000.0
        yaw_req = msg.angular.z

        try:
            limits = calculate_motion_limits(
                self.current_gait_mode,
                self.current_period,
                self.max_step_x_mm,
                self.max_step_y_mm,
                self.max_yaw_step_rad,
            )
            limited = limit_velocity(vx_req, vy_req, yaw_req, limits)
        except (ValueError, ZeroDivisionError) as exc:
            self._cmd_vel_active = False
            self.send_robot_command(format_velocity_command(0.0, 0.0, 0.0))
            self.get_logger().warning(
                f'cmd_vel no válido; robot detenido: {exc}'
            )
            return

        command = format_velocity_command(
            limited.vx_mm_s,
            limited.vy_mm_s,
            limited.yaw_rad_s,
        )
        self._last_cmd_vel_time = time.monotonic()
        self._cmd_vel_active = any(
            value != 0.0
            for value in (limited.vx_mm_s, limited.vy_mm_s, limited.yaw_rad_s)
        )
        self.send_robot_command(command)

    def _check_cmd_vel_timeout(self):
        """Detiene el robot si el último comando de velocidad ha caducado."""
        if not self._cmd_vel_active:
            return

        elapsed = time.monotonic() - self._last_cmd_vel_time
        if elapsed < self.cmd_vel_timeout_s:
            return

        self._cmd_vel_active = False
        self.send_robot_command(format_velocity_command(0.0, 0.0, 0.0))
        self.get_logger().warning('cmd_vel no se ha renovado; robot detenido')

    def robot_command_callback(self, msg):
        """Envía al ESP32 una orden específica del hexápodo."""
        command = msg.data
        if command.startswith('CMD,'):
            return

        if command.startswith('MOD,'):
            parts = command.split(',')
            if len(parts) != 2:
                return

            try:
                mode = GaitMode(int(parts[1]))
            except ValueError:
                return

            if mode != GaitMode.STOP:
                self.current_gait_mode = mode

        elif command.startswith('GAT,'):
            gait_params = parse_gait_command(command)
            if gait_params is None:
                return

            period, height = limit_gait_parameters(
                gait_params[0], gait_params[1], self.current_gait_mode,
            )
            self.current_period = period
            command = format_gait_command(period, height)

        self.send_robot_command(command)

    def send_robot_command(self, command):
        """Envía un comando al ESP32 mediante la UART."""
        self.ser.write(f'{command}\n'.encode())

    # Recepción de telemetría: ESP32 -> ROS 2.

    def read_serial_loop(self):
        """Lee y procesa continuamente las líneas recibidas por UART."""
        while rclpy.ok() and not self._serial_stop_event.is_set():
            try:
                if self.ser.in_waiting <= 0:
                    self._serial_stop_event.wait(0.01)
                    continue

                raw_line = self.ser.readline()
                serial_line = raw_line.decode(
                    'utf-8', errors='ignore',
                ).strip()
            except (OSError, SerialException) as exc:
                if not self._serial_stop_event.is_set():
                    self.get_logger().error(f'Error leyendo serial: {exc}')
                continue

            self._process_serial_line(serial_line)

    def _process_serial_line(self, serial_line):
        """Publica una línea de telemetría según su contenido."""
        telemetry = parse_telemetry_line(serial_line)

        if isinstance(telemetry, OdometryTelemetry):
            self.publish_odom(
                telemetry.x_mm,
                telemetry.y_mm,
                telemetry.yaw_rad,
            )
            self.publish_body_pose(
                telemetry.roll_rad,
                telemetry.pitch_rad,
                telemetry.body_yaw_rad,
                telemetry.body_x_mm,
                telemetry.body_y_mm,
                telemetry.body_z_mm,
            )
            return

        if isinstance(telemetry, ServoHealthTelemetry):
            msg = String()
            msg.data = serial_line
            self.servo_health_pub.publish(msg)
            return

        if isinstance(telemetry, ServoRegisterTelemetry):
            msg = String()
            msg.data = serial_line
            self.servo_register_pub.publish(msg)

    def publish_odom(self, x_mm, y_mm, yaw_rad):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = x_mm / 1000.0
        msg.pose.pose.position.y = y_mm / 1000.0

        quaternion = tf_transformations.quaternion_from_euler(
            0.0,
            0.0,
            yaw_rad,
        )
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        self.odom_pub.publish(msg)

    def publish_body_pose(self, roll, pitch, yaw, x_mm, y_mm, z_mm):
        msg = Pose()
        msg.position.x = x_mm / 1000.0
        msg.position.y = y_mm / 1000.0
        msg.position.z = z_mm / 1000.0
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        self.body_pose_pub.publish(msg)

    # Cierre del nodo y parada del robot.

    def stop_robot(self):
        try:
            self.send_robot_command('CMD,0,0,0')
        except (OSError, SerialException):
            pass

    def destroy_node(self):
        self.get_logger().info('Cerrando puente y deteniendo hexapodo...')
        self._serial_stop_event.set()
        self.serial_thread.join(timeout=0.5)
        self.stop_robot()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
