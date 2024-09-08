import unittest
from unittest.mock import patch, MagicMock
import math
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading, Force
from wander.avoid import AvoidNode

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

    def test_low_force_maintains_heading(self):
        # Set up initial conditions
        force_msg = Force()
        force_msg.magnitude = 0.4  # Below significant_force
        force_msg.direction = 1.0

        heading_msg = Heading()
        heading_msg.distance = 1.0
        heading_msg.angle = 0.5

        # Simulate receiving messages
        self.node.force_callback(force_msg)
        self.node.heading_callback(heading_msg)

        # Check if the original heading was published
        self.node.publisher.publish.assert_called_once()
        published_heading = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_heading, Heading)
        self.assertEqual(published_heading.distance, heading_msg.distance)
        self.assertEqual(published_heading.angle, heading_msg.angle)

    def test_high_force_calculates_new_heading(self):
        # Set up initial conditions
        force_msg = Force()
        force_msg.magnitude = 2.0  # Above significant_force
        force_msg.direction = math.pi / 4

        heading_msg = Heading()
        heading_msg.distance = 1.0
        heading_msg.angle = 0.0

        # Simulate receiving messages
        self.node.force_callback(force_msg)
        self.node.heading_callback(heading_msg)

        # Check if a new heading was published
        self.node.publisher.publish.assert_called_once()
        published_heading = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_heading, Heading)

        # The new heading should be different from the original
        self.assertNotEqual(published_heading.angle, heading_msg.angle)

        # Calculate expected values (simplified version of the node's logic)
        force_factor = self.node.calculate_force_factor()
        expected_x = force_factor * 2.0 * math.cos(math.pi/4) + (1 - force_factor) * 1.0
        expected_y = force_factor * 2.0 * math.sin(math.pi/4)
        expected_distance = math.sqrt(expected_x**2 + expected_y**2)
        expected_angle = math.atan2(expected_y, expected_x)

        # Check if the published heading matches the expected values
        self.assertAlmostEqual(published_heading.distance, expected_distance, places=4)
        self.assertAlmostEqual(published_heading.angle, expected_angle, places=4)

    def test_only_heading_published(self):
        # Set up initial conditions
        heading_msg = Heading()
        heading_msg.distance = 1.5
        heading_msg.angle = -0.5

        # Simulate receiving only a heading message
        self.node.heading_callback(heading_msg)

        # Check if the original heading was published without changes
        self.node.publisher.publish.assert_called_once()
        published_heading = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_heading, Heading)
        self.assertEqual(published_heading.distance, heading_msg.distance)
        self.assertEqual(published_heading.angle, heading_msg.angle)

    def test_collision_force(self):
        # Set up initial conditions for a collision force
        force_msg = Force()
        force_msg.magnitude = 15000.0  # Above collision_force
        force_msg.direction = 3 * math.pi / 4  # Greater than pi/2

        heading_msg = Heading()
        heading_msg.distance = 1.0
        heading_msg.angle = 0.0

        # Simulate receiving messages
        self.node.force_callback(force_msg)
        self.node.heading_callback(heading_msg)

        # Check if a reverse heading was published
        self.node.publisher.publish.assert_called_once()
        published_heading = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_heading, Heading)

        # Check if the published heading matches the expected reverse motion
        self.assertEqual(published_heading.distance, -0.1)
        self.assertEqual(published_heading.angle, force_msg.direction)

    def test_calculate_force_factor(self):
        test_cases = [
            (0.4, 0.064), # Below significant_force
            (0.5, 0.1),  # At significant_force
            (1.75, 0.55),  # Midway
            (3.0, 1.0),  # Maximum
            (4.0, 1.0),  # Above maximum (should be clamped)
        ]

        for force_magnitude, expected_factor in test_cases:
            self.node.current_force.magnitude = force_magnitude
            calculated_factor = self.node.calculate_force_factor()
            self.assertAlmostEqual(calculated_factor, expected_factor, places=4)

if __name__ == '__main__':
    unittest.main()