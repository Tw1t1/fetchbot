import unittest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Force, Heading
from obstacle_avoidance.runaway import RunAwayNode

class TestRunAwayNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RunAwayNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_force_msg_not_triggering_heading(self):
        force_msg = Force()
        force_msg.magnitude = 0.4  # Less than significant_force (0.5)
        force_msg.direction = 1.0

        self.node.publisher = MagicMock()
        self.node.force_callback(force_msg)

        self.node.publisher.publish.assert_not_called()

    def test_force_msg_triggering_heading(self):
        force_msg = Force()
        force_msg.magnitude = 1.0  # Greater than significant_force (0.5)
        force_msg.direction = 1.0

        self.node.publisher = MagicMock()
        self.node.force_callback(force_msg)

        self.node.publisher.publish.assert_called_once()
        published_heading = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_heading, Heading)
        self.assertEqual(published_heading.distance, 1.0)
        self.assertEqual(published_heading.angle, 1.0)

    def test_heading_direction_away_from_obstacles(self):
        force_msg = Force()
        force_msg.magnitude = 15000.0  # Change to float
        force_msg.direction = 3 * 3.14159 / 4  # Greater than pi/2

        self.node.publisher = MagicMock()
        self.node.force_callback(force_msg)

        self.node.publisher.publish.assert_called_once()
        published_heading = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_heading, Heading)
        self.assertEqual(published_heading.distance, -0.1)
        self.assertEqual(published_heading.angle, 3 * 3.14159 / 4)

if __name__ == '__main__':
    unittest.main()