import unittest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading, Force
from wander.avoid import AvoidNode
import math

class TestAvoidNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = AvoidNode()
        self.node.publisher = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    def test_force_callback(self):
        force_msg = Force()
        force_msg.magnitude = 50.0
        force_msg.direction = math.pi / 4
        self.node.force_callback(force_msg)
        self.assertEqual(self.node.current_force, force_msg)

    def test_heading_callback(self):
        heading_msg = Heading()
        heading_msg.distance = 1.0
        heading_msg.angle = math.pi / 2
        with patch.object(self.node, 'calculate_heading') as mock_calculate:
            self.node.heading_callback(heading_msg)
            self.assertEqual(self.node.current_heading, heading_msg)
            mock_calculate.assert_called_once()

    def test_calculate_heading_collision(self):
        self.node.current_force.magnitude = 15000.0
        self.node.current_force.direction = math.pi
        self.node.calculate_heading()
        self.node.publisher.publish.assert_called_once()
        published_msg = self.node.publisher.publish.call_args[0][0]
        self.assertAlmostEqual(published_msg.distance, 0.0)
        self.assertAlmostEqual(published_msg.angle, math.pi)

    def test_calculate_heading_significant_force(self):
        self.node.current_force.magnitude = 1.0
        self.node.current_force.direction = math.pi / 4
        self.node.current_heading.distance = 1.0
        self.node.current_heading.angle = 0.0
        self.node.calculate_heading()
        self.node.publisher.publish.assert_called_once()
        published_msg = self.node.publisher.publish.call_args[0][0]
        self.assertGreater(published_msg.distance, 0)
        self.assertNotEqual(published_msg.angle, 0)

    def test_calculate_heading_insignificant_force(self):
        self.node.current_force.magnitude = 0.1
        self.node.current_heading.distance = 1.0
        self.node.current_heading.angle = math.pi / 6
        self.node.calculate_heading()
        self.node.publisher.publish.assert_called_once()
        published_msg = self.node.publisher.publish.call_args[0][0]
        self.assertEqual(published_msg.distance, 1.0)
        self.assertEqual(published_msg.angle, math.pi / 6)

if __name__ == '__main__':
    unittest.main()