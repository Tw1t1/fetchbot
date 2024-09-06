import unittest, rclpy, math
from unittest.mock import MagicMock
from sensor_msgs.msg import LaserScan
from fetchbot_interfaces.msg import Force, Collision
from obstacle_avoidance.feel_force import FeelForceNode

class TestFeelForceNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = FeelForceNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_scan_callback(self):
        # Create a mock LaserScan message
        mock_scan = LaserScan()
        mock_scan.range_min = 0.1
        mock_scan.range_max = 10.0
        mock_scan.angle_min = -math.pi/2
        mock_scan.angle_max = math.pi/2
        mock_scan.angle_increment = math.pi/4
        mock_scan.ranges = [0.5, 1.0, 2.0, 5.0, 10.0]

        # Create a mock publisher
        mock_publisher = MagicMock()
        self.node.publisher = mock_publisher

        # Call the scan_callback method
        self.node.scan_callback(mock_scan)

        # Check if the publisher was called
        mock_publisher.publish.assert_called_once()

        # Get the published message
        published_msg = mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, Force)

        # Define expected magnitude and direction
        expected_magnitude = 0.005673690604099554
        expected_direction = 1.666779108308315

        # Compare expected and actual values
        self.assertAlmostEqual(published_msg.magnitude, expected_magnitude, places=3)
        self.assertAlmostEqual(published_msg.direction, expected_direction, places=3)

    def test_collision_callback(self):
        # This test remains unchanged
        mock_collision = Collision()
        mock_collision.angle = math.pi/4
        mock_collision.range = 0.1

        self.node.collision_callback(mock_collision)

        self.assertNotEqual(self.node.collision_force_x, 0)
        self.assertNotEqual(self.node.collision_force_y, 0)
        self.assertGreater(self.node.last_collision_time, 0)

if __name__ == '__main__':
    unittest.main()