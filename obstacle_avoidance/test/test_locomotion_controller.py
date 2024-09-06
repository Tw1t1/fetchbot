import unittest, rclpy, math
from unittest.mock import patch
from fetchbot_interfaces.msg import Heading
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
        mock_heading = Heading()
        mock_heading.angle = math.pi/2
        mock_heading.distance = 1.0

        self.node.heading_callback(mock_heading)

        self.assertEqual(self.node.current_heading, mock_heading)
        self.assertTrue(self.node.is_turning)
        self.assertFalse(self.node.is_moving_forward)

    def test_joint_states_callback(self):
        mock_joint_state = JointState()
        mock_joint_state.position = [1.0, 0.0, 2.0]

        self.node.joint_states_callback(mock_joint_state)

        self.assertEqual(self.node.left_wheel_pos, 1.0)
        self.assertEqual(self.node.right_wheel_pos, 2.0)

    def test_calculate_turned_angle(self):
        self.node.initial_left_wheel_pos = 0.0
        self.node.initial_right_wheel_pos = 0.0
        self.node.left_wheel_pos = 1.0
        self.node.right_wheel_pos = 2.0

        angle = self.node.calculate_turned_angle()
        expected_angle = (2.0 - 1.0) * self.node.wheel_radius / self.node.wheel_separation

        self.assertAlmostEqual(angle, expected_angle)

    def test_calculate_moved_distance(self):
        self.node.initial_left_wheel_pos = 0.0
        self.node.initial_right_wheel_pos = 0.0
        self.node.left_wheel_pos = 1.0
        self.node.right_wheel_pos = 1.0

        distance = self.node.calculate_moved_distance()
        expected_distance = (1.0 + 1.0) * self.node.wheel_radius / 2

        self.assertAlmostEqual(distance, expected_distance)

    @patch('obstacle_avoidance.locomotion_controller.LocomotionControllerNode.check_if_stuck')
    @patch('obstacle_avoidance.locomotion_controller.LocomotionControllerNode.handle_stuck')
    @patch('obstacle_avoidance.locomotion_controller.LocomotionControllerNode.handle_turning')
    @patch('obstacle_avoidance.locomotion_controller.LocomotionControllerNode.handle_moving_forward')
    def test_timer_callback(self, mock_forward, mock_turning, mock_stuck, mock_check_stuck):
        mock_check_stuck.return_value = False
        self.node.current_heading = Heading()
        self.node.is_turning = True

        self.node.timer_callback()

        mock_turning.assert_called_once()
        mock_forward.assert_not_called()
        mock_stuck.assert_not_called()

    def test_check_if_stuck(self):
        self.node.initial_left_wheel_pos = 0.0
        self.node.initial_right_wheel_pos = 0.0
        self.node.left_wheel_pos = 0.0
        self.node.right_wheel_pos = 0.0
        self.node.last_movement_time = self.node.get_clock().now()

        # Should not be stuck initially
        self.assertFalse(self.node.check_if_stuck())

        # Simulate being stuck for longer than the threshold
        self.node.last_movement_time = self.node.get_clock().now() - rclpy.duration.Duration(seconds=self.node.stuck_time_threshold + 1)

        self.assertTrue(self.node.check_if_stuck())

if __name__ == '__main__':
    unittest.main()
