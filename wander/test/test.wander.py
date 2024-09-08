import unittest
from unittest.mock import patch, MagicMock
import math
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
from wander.wander import WanderNode  # Adjust the import path as needed

class TestWanderNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = WanderNode()
        self.node.publisher = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    @patch('random.uniform')
    def test_timer_callback_publishes_random_heading(self, mock_uniform):
        # Test multiple iterations
        iterations = 5
        mock_uniform.side_effect = [0.5, -0.3, 0.1, -0.7, 0.9]  # Predefined random values

        for i in range(iterations):
            self.node.timer_callback()

            # Check if a message was published
            self.node.publisher.publish.assert_called()
            published_heading = self.node.publisher.publish.call_args[0][0]
            self.assertIsInstance(published_heading, Heading)

            # Check if distance is constant
            self.assertEqual(published_heading.distance, self.node.distance)

            # Check if angle is within the expected range
            self.assertLessEqual(abs(published_heading.angle), self.node.max_angle)

            # Check if the angle follows the smoothing logic
            expected_angle = self.node.factor * self.node.last_angle + (1 - self.node.factor) * mock_uniform.side_effect[i] * self.node.max_angle
            self.assertAlmostEqual(published_heading.angle, expected_angle, places=6)

            # Update last_angle for the next iteration
            self.node.last_angle = published_heading.angle

            # Reset the mock for the next iteration
            self.node.publisher.publish.reset_mock()

    def test_initial_values(self):
        self.assertEqual(self.node.distance, 1.0)
        self.assertEqual(self.node.last_angle, 0.0)
        self.assertAlmostEqual(self.node.factor, 0.7)
        self.assertAlmostEqual(self.node.max_angle, math.pi * 0.7)

if __name__ == '__main__':
    unittest.main()