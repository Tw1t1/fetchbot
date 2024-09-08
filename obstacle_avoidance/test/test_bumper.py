import unittest
import sys
import rclpy
from unittest.mock import MagicMock, patch
from fetchbot_interfaces.msg import Collision
from obstacle_avoidance.bumper import BumperNode

# Mock RPi.GPIO
sys.modules['RPi'] = MagicMock()
sys.modules['RPi.GPIO'] = MagicMock()

class TestBumperNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = BumperNode()

    def tearDown(self):
        self.node.destroy_node()

    @patch('obstacle_avoidance.bumper.GPIO.input')
    def test_read_bumper_sensors(self, mock_gpio_input):
        # Test both bumpers not pressed
        mock_gpio_input.side_effect = [1, 1]
        self.assertEqual(self.node.read_bumper_sensors(), (False, False))

        # Test left bumper pressed
        mock_gpio_input.side_effect = [0, 1]
        self.assertEqual(self.node.read_bumper_sensors(), (True, False))

        # Test right bumper pressed
        mock_gpio_input.side_effect = [1, 0]
        self.assertEqual(self.node.read_bumper_sensors(), (False, True))

        # Test both bumpers pressed
        mock_gpio_input.side_effect = [0, 0]
        self.assertEqual(self.node.read_bumper_sensors(), (True, True))

        # Reset side_effect
        mock_gpio_input.side_effect = None

    def test_calculate_collision_angle(self):
        # Test both bumpers pressed
        angle, range_ = self.node.calculate_collision_angle(True, True)
        self.assertEqual(angle, self.node.rear_angle)
        self.assertEqual(range_, min(self.node.left_range, self.node.right_range))

        # Test left bumper pressed
        angle, range_ = self.node.calculate_collision_angle(True, False)
        self.assertEqual(angle, self.node.left_angle)
        self.assertEqual(range_, self.node.left_range)

        # Test right bumper pressed
        angle, range_ = self.node.calculate_collision_angle(False, True)
        self.assertEqual(angle, self.node.right_angle)
        self.assertEqual(range_, self.node.right_range)

        # Test no bumpers pressed
        angle, range_ = self.node.calculate_collision_angle(False, False)
        self.assertIsNone(angle)
        self.assertIsNone(range_)

    @patch('obstacle_avoidance.bumper.BumperNode.read_bumper_sensors')
    @patch('obstacle_avoidance.bumper.BumperNode.calculate_collision_angle')
    def test_timer_callback(self, mock_calculate, mock_read):
        mock_read.return_value = (True, False)
        mock_calculate.return_value = (1.5, 0.5)

        # Create a mock publisher
        mock_publisher = MagicMock()
        self.node.collision_pub = mock_publisher

        self.node.timer_callback()

        # Check if the publisher was called with the correct message
        mock_publisher.publish.assert_called_once()
        published_msg = mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, Collision)
        self.assertEqual(published_msg.angle, 1.5)
        self.assertEqual(published_msg.range, 0.5)

if __name__ == '__main__':
    unittest.main()