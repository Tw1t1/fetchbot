import unittest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
from wander.wander import WanderNode
import math

class TestWanderNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = WanderNode()

    def tearDown(self):
        self.node.destroy_node()

    @patch('random.uniform')
    def test_timer_callback(self, mock_uniform):
        mock_uniform.return_value = 0.5  # Mocking random.uniform to return 0.5
        self.node.timer_callback()
        last_published = self.node.publisher.publish.call_args[0][0]
        
        self.assertIsInstance(last_published, Heading)
        self.assertEqual(last_published.distance, 1.5)
        self.assertEqual(last_published.angle, 0.5)
        mock_uniform.assert_called_once_with(math.pi/2, -math.pi/2)

    def test_publish_frequency(self):
        self.assertEqual(self.node.timer.timer_period_ns, 1_000_000_000)  # 1 second in nanoseconds

if __name__ == '__main__':
    unittest.main()