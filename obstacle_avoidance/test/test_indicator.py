import unittest
from unittest.mock import MagicMock, patch
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fetchbot_interfaces.msg import Collision
from obstacle_avoidance.indicator import IndicatorNode

class TestIndicatorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = IndicatorNode()
        self.node.pwm_r = MagicMock()
        self.node.pwm_g = MagicMock()
        self.node.pwm_b = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    def test_collision_callback(self):
        mock_collision = Collision()
        self.node.collision_callback(mock_collision)
        self.assertTrue(self.node.status['collision']['active'])
        self.assertAlmostEqual(self.node.status['collision']['timestamp'], time.time(), delta=0.1)

    def test_follow_ball_callback(self):
        mock_string = String()
        mock_string.data = "UNREACHABLE"
        self.node.follow_ball_callback(mock_string)
        self.assertTrue(self.node.status['follow_ball']['active'])
        
        self.node.status['follow_ball']['active'] = False
        
        mock_string.data = "REACHED"
        self.node.follow_ball_callback(mock_string)
        self.assertFalse(self.node.status['follow_ball']['active'])

    def test_orient_home_callback(self):
        mock_string = String()
        self.node.orient_home_callback(mock_string)
        self.assertTrue(self.node.status['orient_home']['active'])
        self.assertAlmostEqual(self.node.status['orient_home']['timestamp'], time.time(), delta=0.1)

    def test_low_battery_callback(self):
        mock_string = String()
        self.node.low_battery_callback(mock_string)
        self.assertTrue(self.node.status['low_battery']['active'])
        self.assertAlmostEqual(self.node.status['low_battery']['timestamp'], time.time(), delta=0.1)

    def test_set_color(self):
        self.node.set_color(255, 128, 64)
        self.node.pwm_r.ChangeDutyCycle.assert_called_with(100)
        self.node.pwm_g.ChangeDutyCycle.assert_called_with(50.19607843137255)
        self.node.pwm_b.ChangeDutyCycle.assert_called_with(25.098039215686274)

    @patch('obstacle_avoidance.indicator.IndicatorNode.set_color')
    def test_update_led(self, mock_set_color):
        self.node.status['collision']['active'] = True
        self.node.status['follow_ball']['active'] = True
        self.node.status['collision']['timestamp'] = time.time()
        self.node.status['follow_ball']['timestamp'] = time.time()
        self.node.update_led()
        mock_set_color.assert_called_once()

    @patch('obstacle_avoidance.indicator.IndicatorNode.publish_status')
    def test_update_status(self, mock_publish):
        self.node.update_status('collision')
        self.assertTrue(self.node.status['collision']['active'])
        self.assertAlmostEqual(self.node.status['collision']['timestamp'], time.time(), delta=0.1)
        mock_publish.assert_called_once()

    def test_publish_status(self):
        self.node.status['collision']['active'] = True
        self.node.status['follow_ball']['active'] = True
        self.node.publisher = MagicMock()
        
        self.node.publish_status()
        
        self.node.publisher.publish.assert_called_once()
        published_msg = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, String)
        self.assertEqual(published_msg.data, 'collision follow_ball')

if __name__ == '__main__':
    unittest.main()