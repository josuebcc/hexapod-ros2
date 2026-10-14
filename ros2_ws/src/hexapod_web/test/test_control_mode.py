import unittest
from unittest.mock import Mock

from hexapod_web.web_app import create_app


class TestControlModeApi(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.client = create_app(self.node).test_client()

    def test_mode_state(self):
        self.node.get_control_mode.return_value = {'available': True, 'mode': 'manual'}
        self.assertEqual(self.client.get('/control_mode').json['mode'], 'manual')

    def test_confirmed_switch(self):
        self.node.set_control_mode.return_value = {'ok': True, 'mode': 'autonomous'}
        response = self.client.post('/control_mode', json={'mode': 'autonomous'})
        self.assertEqual(response.status_code, 200)
        self.node.set_control_mode.assert_called_once_with('autonomous')

    def test_invalid_mode(self):
        for payload in ({}, [], {'mode': 'invalid'}, {'mode': True}):
            self.assertEqual(self.client.post('/control_mode', json=payload).status_code, 400)
        self.node.set_control_mode.assert_not_called()

    def test_avoider_unavailable(self):
        self.node.set_control_mode.return_value = {'ok': False, 'error': 'Sin conexion'}
        response = self.client.post('/control_mode', json={'mode': 'autonomous'})
        self.assertEqual(response.status_code, 503)

    def test_manual_command_rejected_in_autonomy(self):
        self.node.send_velocity.return_value = False
        response = self.client.post('/cmd_vel', json={'linear_x': 0.1})
        self.assertEqual(response.status_code, 409)


if __name__ == '__main__':
    unittest.main()
