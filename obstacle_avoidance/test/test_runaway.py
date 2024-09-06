import unittest, rclpy, math
from unittest.mock import MagicMock
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

    def test_force_callback_significant_force(self):
        mock_force = Force()
        mock_force.magnitude = 1.0  # Greater than self.significant_force
        mock_force.direction = math.pi/4

        # Create a mock publisher
        mock_publisher = MagicMock()
        self.node.publisher = mock_publisher

        self.node.force_callback(mock_force)

        # Check if the publisher was called
        mock_publisher.publish.assert_called_once()

        # Check the published message
        published_msg = mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, Heading)
        self.assertEqual(published_msg.distance, 1.0)
        self.assertEqual(published_msg.angle, math.pi/4)

    def test_force_callback_collision_force(self):
        mock_force = Force()
        mock_force.magnitude = 15000.0  # Greater than self.collision_force
        mock_force.direction = math.pi  # The obsticals are in front of the robot

        # Create a mock publisher
        mock_publisher = MagicMock()
        self.node.publisher = mock_publisher

        self.node.force_callback(mock_force)

        # Check if the publisher was called
        mock_publisher.publish.assert_called_once()

        # Check the published message
        published_msg = mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, Heading)
        self.assertEqual(published_msg.distance, 0.0)
        self.assertEqual(published_msg.angle, math.pi)

    def test_force_callback_insignificant_force(self):
        mock_force = Force()
        mock_force.magnitude = 0.1  # Less than self.significant_force
        mock_force.direction = math.pi/4

        # Create a mock publisher
        mock_publisher = MagicMock()
        self.node.publisher = mock_publisher

        self.node.force_callback(mock_force)

        # Check that the publisher was not called
        mock_publisher.publish.assert_not_called()

if __name__ == '__main__':
    unittest.main()
