import math

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SectorObstacleAvoider(Node):
    """Evitador reactivo sencillo basado en sectores del LiDAR."""

    def __init__(self):
        super().__init__('reactive_node')

        # Cola de 1: solo interesa el barrido y la propuesta más recientes.
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.lidar_callback,
            1,
        )
        self.drive_pub = self.create_publisher(
            Twist,
            '/obstacle_avoider/cmd_vel',
            1,
        )

        # --- Parámetros (distancias en m, velocidades en m/s y rad/s) ---

        self.front_clearance = 0.50     # Distancia libre mínima delante para empezar a avanzar.
        self.resume_clearance = 0.60    # Distancia libre mínima delante para dejar de girar y volver a avanzar. 
        self.robot_radius = 0.35        # Círculo que envuelve el robot: medio ancho del pasillo frontal y espacio libre necesario para girar.
        self.side_influence = 0.80      # Al avanzar, las paredes laterales más cercanas que esta distancia desvían el rumbo hacia el otro lado.
        self.forward_speed = 0.08       # Velocidad lineal al avanzar.
        self.turn_speed = 0.25          # Velocidad angular al girar.

        # Sectores laterales: de 35 a 120 grados a cada lado.
        self.front_half_angle = math.radians(35.0)
        self.side_limit_angle = math.radians(120.0)

        # --- Estado ---
        self.turn_direction = 0.0  # Sentido del giro en curso: +1 izquierda, -1 derecha, 0 avanzando.

    def sector_clearance(self, ranges, angles, start_angle, end_angle):
        """Distancia más cercana dentro de un sector angular."""
        in_sector = (angles >= start_angle) & (angles <= end_angle)
        values = ranges[in_sector & np.isfinite(ranges)]
        if values.size == 0:
            return 0.0

        return float(np.min(values))

    def corridor_clearance(self, ranges, angles, half_width):
        """Distancia libre hacia delante en un pasillo del ancho del robot."""
        valid = np.isfinite(ranges)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        ahead = (x > 0.0) & (np.abs(y) <= half_width)
        if not ahead.any():
            return 0.0

        return float(np.min(x[ahead]))

    def lidar_callback(self, data):
        """Decide en cada barrido si avanzar, girar o quedarse parado."""
        
        drive_msg = Twist()
        ranges = np.asarray(data.ranges, dtype=float)

        if (
            ranges.size == 0
            or not math.isfinite(data.angle_increment)
            or data.angle_increment == 0.0
        ):
            self.drive_pub.publish(drive_msg)
            return

        angles = data.angle_min + np.arange(ranges.size) * data.angle_increment  # Array de ángulos de cada medida del LiDAR.

        # Frente: distancia libre en el pasillo que barre el robot al avanzar. Laterales: medida más cercana de +35° a +120° (izquierda) y de −120° a −35° (derecha).
        front = self.corridor_clearance(ranges, angles, self.robot_radius)
        left = self.sector_clearance(
            ranges, angles, self.front_half_angle, self.side_limit_angle)
        right = self.sector_clearance(
            ranges, angles, -self.side_limit_angle, -self.front_half_angle)
        
        # Para girar sobre el centro, el círculo del robot debe quedar libre en todas las direcciones.
        surrounding = self.sector_clearance(ranges, angles, -math.pi, math.pi)

        # Mientras gira, exige más espacio delante para volver a avanzar.
        if self.turn_direction == 0.0:
            needed = self.front_clearance
        else:
            needed = self.resume_clearance

        if front >= needed:
            # Avanza separándose de las paredes laterales cercanas.
            self.turn_direction = 0.0
            left_near = min(left, self.side_influence)
            right_near = min(right, self.side_influence)
            drive_msg.linear.x = self.forward_speed
            drive_msg.angular.z = ((left_near - right_near) / self.side_influence * self.turn_speed)
        elif surrounding >= self.robot_radius:
            # Gira sobre sí mismo hasta que el pasillo de delante vuelve a estar libre. El lado se elige una vez y se mantiene hasta volver a avanzar.
            if self.turn_direction == 0.0:
                self.turn_direction = 1.0 if left >= right else -1.0
            drive_msg.angular.z = self.turn_direction * self.turn_speed
        # Si hay algo dentro del círculo del robot, no gira y la velocidad queda a cero.

        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    reactive_node = SectorObstacleAvoider()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
