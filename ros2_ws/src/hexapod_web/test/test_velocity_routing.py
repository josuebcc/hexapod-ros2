import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import Twist
from hexapod_web.web_node import WebPanelNode


class TestVelocityRouting(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            control_mode='manual',
            control_mode_lock=threading.Lock(),
            last_avoider_time=None,
            cmd_vel_pub=Mock(),
        )
        self.node.set_control_mode = lambda mode: WebPanelNode.set_control_mode(self.node, mode)
        self.msg = Twist()
        self.msg.linear.x = 0.2

    def receive_avoider(self):
        WebPanelNode.avoider_velocity_cb(self.node, self.msg)

    def test_manual_works_without_avoider(self):
        self.assertTrue(WebPanelNode.send_velocity(self.node, 0.1, 0.0, 0.0))
        self.assertEqual(self.node.cmd_vel_pub.publish.call_args.args[0].linear.x, 0.1)

    def test_manual_ignores_avoider(self):
        self.receive_avoider()
        self.node.cmd_vel_pub.publish.assert_not_called()

    def test_autonomy_forwards_new_proposals_only(self):
        self.receive_avoider()
        self.assertTrue(self.node.set_control_mode('autonomous')['ok'])
        self.node.cmd_vel_pub.publish.assert_called_once_with(Twist())
        self.node.cmd_vel_pub.publish.reset_mock()
        self.assertFalse(WebPanelNode.send_velocity(self.node, 0.1, 0.0, 0.0))
        self.node.cmd_vel_pub.publish.assert_not_called()
        self.receive_avoider()
        self.node.cmd_vel_pub.publish.assert_called_once_with(self.msg)

    def test_reject_missing_and_stale_proposals(self):
        self.assertFalse(self.node.set_control_mode('autonomous')['ok'])
        self.node.last_avoider_time = 1.0
        with patch('hexapod_web.web_node.time.monotonic', return_value=2.0):
            self.assertFalse(self.node.set_control_mode('autonomous')['ok'])
        self.assertEqual(self.node.control_mode, 'manual')

    def test_stop_restores_manual(self):
        self.node.control_mode = 'autonomous'
        WebPanelNode._update_robot_state_from_command(self.node, 'MOD,0')
        self.assertEqual(self.node.control_mode, 'manual')
        self.node.cmd_vel_pub.publish.assert_called_once_with(Twist())
        self.node.cmd_vel_pub.publish.reset_mock()
        self.receive_avoider()
        self.node.cmd_vel_pub.publish.assert_not_called()

    def test_invalid_proposal_stops_in_autonomy(self):
        self.node.control_mode = 'autonomous'
        self.msg.angular.z = float('nan')
        self.receive_avoider()
        self.node.cmd_vel_pub.publish.assert_called_once_with(Twist())
        self.assertIsNone(self.node.last_avoider_time)


if __name__ == '__main__':
    unittest.main()
