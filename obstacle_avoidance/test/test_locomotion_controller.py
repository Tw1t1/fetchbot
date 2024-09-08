import unittest
from unittest.mock import MagicMock, patch
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from fetchbot_interfaces.msg import Heading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from obstacle_avoidance.locomotion_controller import LocomotionControllerNode

class TestLocomotionControllerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = LocomotionControllerNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_heading_callback(self):
        heading_msg = Heading()
        heading_msg.distance = 1.0
        heading_msg.angle = 0.5

        self.node.heading_callback(heading_msg)

        self.assertEqual(self.node.current_heading, heading_msg)
        self.assertFalse(self.node.obstacle_detected)

        heading_msg.distance = -0.1
        self.node.heading_callback(heading_msg)
        self.assertTrue(self.node.obstacle_detected)

    def test_joint_states_callback(self):
        joint_state_msg = JointState()
        joint_state_msg.position = [1.0, 2.0, 3.0]

        self.node.joint_states_callback(joint_state_msg)

        self.assertEqual(self.node.left_wheel_pos, 1.0)
        self.assertEqual(self.node.right_wheel_pos, 3.0)

    @patch('obstacle_avoidance.locomotion_controller.LocomotionControllerNode.get_clock')
    def test_timer_callback_timeout(self, mock_get_clock):
        mock_clock = MagicMock()
        mock_clock.now.return_value = Time(seconds=10)
        mock_get_clock.return_value = mock_clock

        self.node.last_heading_time = Time(seconds=9)
        self.node.publisher = MagicMock()
        self.node.timer_callback()

        self.node.publisher.publish.assert_called_once()
        published_twist = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_twist, Twist)
        self.assertEqual(published_twist.linear.x, 0.0)
        self.assertEqual(published_twist.angular.z, 0.0)

    def test_calculate_desired_linear_velocity(self):
        self.node.current_heading = Heading()
        self.node.current_heading.distance = 1.0
        self.node.current_heading.angle = 0.0

        velocity = self.node.calculate_desired_linear_velocity()
        self.assertAlmostEqual(velocity, 0.4)  # max_linear_velocity

        self.node.current_heading.distance = 0.1
        velocity = self.node.calculate_desired_linear_velocity()
        self.assertEqual(velocity, 0.0)  # stopping distance

    def test_calculate_desired_angular_velocity(self):
        self.node.current_heading = Heading()
        self.node.current_heading.angle = math.pi / 2

        velocity = self.node.calculate_desired_angular_velocity()
        self.assertAlmostEqual(velocity, 1.0)  # angular_velocity

        self.node.current_heading.angle = 0.05
        velocity = self.node.calculate_desired_angular_velocity()
        self.assertEqual(velocity, 0.0)  # small angle threshold

    def test_stop_robot(self):
        self.node.publisher = MagicMock()
        self.node.stop_robot()

        self.node.publisher.publish.assert_called_once()
        published_twist = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_twist, Twist)
        self.assertEqual(published_twist.linear.x, 0.0)
        self.assertEqual(published_twist.angular.z, 0.0)
        self.assertEqual(self.node.last_action, 'none')

    @patch('obstacle_avoidance.locomotion_controller.LocomotionControllerNode.get_clock')
    def test_heading_to_twist(self, mock_get_clock):
        mock_clock = MagicMock()
        mock_clock.now.return_value = Time(seconds=10)
        mock_get_clock.return_value = mock_clock

        heading_msg = Heading()
        heading_msg.distance = 1.0
        heading_msg.angle = math.pi / 4

        self.node.heading_callback(heading_msg)
        self.node.publisher = MagicMock()
        self.node.timer_callback()

        self.node.publisher.publish.assert_called_once()
        published_twist = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_twist, Twist)

        expected_linear_velocity = min(self.node.max_linear_velocity, heading_msg.distance)
        expected_linear_velocity *= (1 - 0.5 * (math.pi/4) / math.pi)
        self.assertAlmostEqual(published_twist.linear.x, expected_linear_velocity, places=2)

        expected_angular_velocity = min(self.node.angular_velocity, abs(heading_msg.angle))
        self.assertAlmostEqual(published_twist.angular.z, expected_angular_velocity, places=2)

    @patch('obstacle_avoidance.locomotion_controller.LocomotionControllerNode.get_clock')
    def test_obstacle_avoidance(self, mock_get_clock):
        mock_clock = MagicMock()
        mock_clock.now.return_value = Time(seconds=10)
        mock_get_clock.return_value = mock_clock

        heading_msg = Heading()
        heading_msg.distance = -0.1
        heading_msg.angle = math.pi / 4

        self.node.heading_callback(heading_msg)
        self.node.publisher = MagicMock()
        self.node.timer_callback()

        self.node.publisher.publish.assert_called_once()
        published_twist = self.node.publisher.publish.call_args[0][0]
        self.assertIsInstance(published_twist, Twist)

        self.assertEqual(published_twist.linear.x, self.node.reverse_speed)
        self.assertEqual(published_twist.angular.z, 0.0)
        self.assertEqual(self.node.last_action, 'reverse')

        self.node.publisher.publish.reset_mock()
        self.node.timer_callback()