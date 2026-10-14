import math
import unittest
from unittest.mock import patch

from geometry_msgs.msg import Twist
from obstacle_avoider.reactive_node import SectorObstacleAvoider
import rclpy
from sensor_msgs.msg import LaserScan


class TestSectorObstacleAvoider(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SectorObstacleAvoider()
        self.scan = LaserScan()
        self.scan.angle_min = -math.pi
        self.scan.angle_increment = 2.0 * math.pi / 1080
        self.scan.range_min = 0.12
        self.scan.range_max = 12.0
        self.scan.ranges = [3.0] * 1081

    def tearDown(self):
        self.node.destroy_node()

    def sector_scan(self, front=3.0, left=3.0, right=3.0, rear=3.0):
        """Pared delante a x = front y paredes paralelas a y = left / y = -right."""
        ranges = []
        for index in range(1081):
            angle = self.scan.angle_min + index * self.scan.angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if -self.node.front_half_angle <= angle <= self.node.front_half_angle:
                ranges.append(front / math.cos(angle))
            elif self.node.front_half_angle < angle <= self.node.side_limit_angle:
                ranges.append(left / math.sin(angle))
            elif -self.node.side_limit_angle <= angle < -self.node.front_half_angle:
                ranges.append(right / -math.sin(angle))
            else:
                ranges.append(rear)
        return ranges

    def ray_index(self, degrees):
        return round((math.radians(degrees) - self.scan.angle_min) / self.scan.angle_increment)

    def publish_scan(self, ranges):
        self.scan.ranges = ranges
        with patch.object(self.node.drive_pub, 'publish') as publish:
            self.node.lidar_callback(self.scan)
        return publish.call_args.args[0]

    def assert_components(self, msg, speed, yaw):
        self.assertIsInstance(msg, Twist)
        self.assertAlmostEqual(msg.linear.x, speed)
        self.assertAlmostEqual(msg.angular.z, yaw)
        self.assertEqual(
            (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y),
            (0.0, 0.0, 0.0, 0.0),
        )

    def test_clear_front_moves_straight(self):
        msg = self.publish_scan(self.sector_scan())
        self.assert_components(msg, self.node.forward_speed, 0.0)

    def test_front_obstacle_turns_left_when_left_is_clearer(self):
        msg = self.publish_scan(self.sector_scan(front=0.42, left=2.0, right=1.0))
        self.assert_components(msg, 0.0, self.node.turn_speed)

    def test_front_obstacle_turns_right_when_right_is_clearer(self):
        msg = self.publish_scan(self.sector_scan(front=0.42, left=1.0, right=2.0))
        self.assert_components(msg, 0.0, -self.node.turn_speed)

    def test_equal_sides_turn_left(self):
        msg = self.publish_scan(self.sector_scan(front=0.42, left=1.5, right=1.5))
        self.assert_components(msg, 0.0, self.node.turn_speed)

    def test_turn_direction_is_kept_until_front_is_clear(self):
        first = self.publish_scan(self.sector_scan(front=0.42, left=2.0, right=1.0))
        later = self.publish_scan(self.sector_scan(front=0.42, left=1.0, right=2.0))

        self.assert_components(first, 0.0, self.node.turn_speed)
        self.assert_components(later, 0.0, self.node.turn_speed)

    def test_turning_needs_resume_clearance_to_advance_again(self):
        self.publish_scan(self.sector_scan(front=0.42, left=2.0, right=1.0))
        between = self.publish_scan(self.sector_scan(front=0.55))
        clear = self.publish_scan(self.sector_scan(front=0.65))

        self.assert_components(between, 0.0, self.node.turn_speed)
        self.assert_components(clear, self.node.forward_speed, 0.0)

    def test_side_is_chosen_again_after_advancing(self):
        self.publish_scan(self.sector_scan(front=0.42, left=2.0, right=1.0))
        self.publish_scan(self.sector_scan(front=0.65))
        msg = self.publish_scan(self.sector_scan(front=0.42, left=1.0, right=2.0))

        self.assert_components(msg, 0.0, -self.node.turn_speed)

    def test_wall_on_the_right_steers_left_while_advancing(self):
        msg = self.publish_scan(self.sector_scan(right=0.40))
        steer = (0.80 - 0.40) / self.node.side_influence * self.node.turn_speed
        self.assert_components(msg, self.node.forward_speed, steer)

    def test_wall_on_the_left_steers_right_while_advancing(self):
        msg = self.publish_scan(self.sector_scan(left=0.40))
        steer = (0.80 - 0.40) / self.node.side_influence * self.node.turn_speed
        self.assert_components(msg, self.node.forward_speed, -steer)

    def test_far_walls_do_not_steer(self):
        msg = self.publish_scan(self.sector_scan(left=1.0, right=2.0))
        self.assert_components(msg, self.node.forward_speed, 0.0)

    def test_close_obstacle_prevents_unsafe_rotation(self):
        msg = self.publish_scan(self.sector_scan(front=0.20, left=2.0, right=1.0))
        self.assert_components(msg, 0.0, 0.0)

    def test_turns_when_obstacles_are_outside_the_robot_circle(self):
        msg = self.publish_scan(self.sector_scan(front=0.42, left=0.38, right=2.0))
        self.assert_components(msg, 0.0, -self.node.turn_speed)

    def test_obstacle_in_front_of_a_front_leg_blocks_advance(self):
        # A 50° y 0,40 m (x = 0,26, y = 0,31): fuera del antiguo cono de ±35°,
        # pero dentro del pasillo que barre el robot al avanzar.
        ranges = self.sector_scan()
        start = self.ray_index(48.0)
        ranges[start:self.ray_index(52.0) + 1] = [0.40] * (self.ray_index(52.0) + 1 - start)
        msg = self.publish_scan(ranges)
        self.assert_components(msg, 0.0, -self.node.turn_speed)

    def test_passage_wider_than_the_robot_is_crossed(self):
        msg = self.publish_scan(self.sector_scan(left=0.40, right=0.40))
        self.assert_components(msg, self.node.forward_speed, 0.0)

    def test_passage_narrower_than_the_robot_is_not_entered(self):
        msg = self.publish_scan(self.sector_scan(left=0.30, right=0.30))
        self.assert_components(msg, 0.0, 0.0)

    def test_one_short_return_is_an_obstacle(self):
        ranges = self.sector_scan(left=1.0, right=2.0)
        ranges[len(ranges) // 2] = 0.42
        msg = self.publish_scan(ranges)
        self.assert_components(msg, 0.0, -self.node.turn_speed)

    def test_scan_without_echo_stops(self):
        msg = self.publish_scan([float('inf')] * 1081)
        self.assert_components(msg, 0.0, 0.0)

    def test_invalid_scan_stops(self):
        msg = self.publish_scan([float('nan')] * 1081)
        self.assert_components(msg, 0.0, 0.0)

    def test_zero_angle_increment_stops(self):
        self.scan.angle_increment = 0.0
        msg = self.publish_scan(self.sector_scan())
        self.assert_components(msg, 0.0, 0.0)

    def test_robot_is_a_35_cm_circle(self):
        self.assertAlmostEqual(self.node.robot_radius, 0.35)

    def test_input_is_filtered_scan(self):
        self.assertEqual(self.node.lidar_sub.topic_name, '/scan_filtered')

    def test_output_is_proposal_topic(self):
        self.assertEqual(self.node.drive_pub.topic_name, '/obstacle_avoider/cmd_vel')


if __name__ == '__main__':
    unittest.main()
