import rclpy
import math
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class LocomotionControllerNode(Node):
    def __init__(self):
        super().__init__('locomotion_controller')
        self.subscription = self.create_subscription(
            Heading,
            'avoid_runaway_suppressor',
            self.heading_callback,
            2)
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.joint_states_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        self.linear_velocity = 0.5
        self.angular_velocity = 1.0
        self.current_heading = None
        self.is_turning = False
        self.is_moving_forward = False

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.initial_left_wheel_pos = 0.0
        self.initial_right_wheel_pos = 0.0
        self.wheel_separation = 0.385  # Adjust this to your robot's wheel separation
        self.wheel_radius = 0.0525  # Adjust this to your robot's wheel radius

        self.stuck_threshold = 0.001  # Minimum change in position to consider not stuck
        self.stuck_time_threshold = 2.0  # Time in seconds to consider robot stuck
        self.last_movement_time = self.get_clock().now()
        self.is_stuck = False
        self.unstuck_distance = 0.1  # Distance to move back when stuck

    def heading_callback(self, msg):
        self.current_heading = msg
        self.is_turning = True
        self.is_moving_forward = False
        self.initial_left_wheel_pos = self.left_wheel_pos
        self.initial_right_wheel_pos = self.right_wheel_pos
        self.reset_stuck_detection()

    def joint_states_callback(self, msg):
        self.left_wheel_pos = msg.position[0]
        self.right_wheel_pos = msg.position[2]

    def timer_callback(self):
        if self.current_heading is None:
            return

        twist = Twist()

        if self.check_if_stuck():
            self.handle_stuck(twist)
        elif self.is_turning:
            self.handle_turning(twist)
        elif self.is_moving_forward:
            self.handle_moving_forward(twist)

        self.publisher.publish(twist)

    def handle_turning(self, twist):
        target_angle = self.current_heading.angle
        current_angle = self.calculate_turned_angle()

        if abs(current_angle) >= abs(target_angle):
            twist.angular.z = 0.0
            self.is_turning = False
            self.is_moving_forward = True
            self.initial_left_wheel_pos = self.left_wheel_pos
            self.initial_right_wheel_pos = self.right_wheel_pos
            self.reset_stuck_detection()
        else:
            direction = 1.0 if target_angle >= 0 else -1.0
            twist.angular.z = direction * self.angular_velocity

    def handle_moving_forward(self, twist):
        target_distance = self.current_heading.distance
        current_distance = self.calculate_moved_distance()

        if current_distance >= target_distance:
            twist.linear.x = 0.0
            self.is_moving_forward = False
            self.current_heading = None
            self.reset_stuck_detection()
        else:
            twist.linear.x = self.linear_velocity

    def calculate_turned_angle(self):
        left_wheel_rotation = self.left_wheel_pos - self.initial_left_wheel_pos
        right_wheel_rotation = self.right_wheel_pos - self.initial_right_wheel_pos
        self.get_logger().info(f'Left wheel: {left_wheel_rotation}, Right wheel: {right_wheel_rotation}')
        return (right_wheel_rotation - left_wheel_rotation) * self.wheel_radius / self.wheel_separation

    def calculate_moved_distance(self):
        left_wheel_rotation = self.left_wheel_pos - self.initial_left_wheel_pos
        right_wheel_rotation = self.right_wheel_pos - self.initial_right_wheel_pos
        self.get_logger().info(f'Left wheel: {left_wheel_rotation}, Right wheel: {right_wheel_rotation}')
        return (left_wheel_rotation + right_wheel_rotation) * self.wheel_radius / 2

    def check_if_stuck(self):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_movement_time).nanoseconds / 1e9

        left_diff = abs(self.left_wheel_pos - self.initial_left_wheel_pos)
        right_diff = abs(self.right_wheel_pos - self.initial_right_wheel_pos)

        if (left_diff < self.stuck_threshold and right_diff < self.stuck_threshold
                and time_diff > self.stuck_time_threshold):
            self.is_stuck = True
            return True
        
        if left_diff > self.stuck_threshold or right_diff > self.stuck_threshold:
            self.last_movement_time = current_time

        return False

    def handle_stuck(self, twist):
        self.get_logger().warn('Robot is stuck. Moving backwards to unstuck.')
        twist.linear.x = -self.linear_velocity / 2  # Move backwards at half speed
        
        # Check if we've moved back enough to consider ourselves unstuck
        if abs(self.left_wheel_pos - self.initial_left_wheel_pos) > self.unstuck_distance or \
           abs(self.right_wheel_pos - self.initial_right_wheel_pos) > self.unstuck_distance:
            self.is_stuck = False
            self.reset_stuck_detection()
            self.get_logger().info('Robot unstuck. Resuming normal operation.')

    def reset_stuck_detection(self):
        self.last_movement_time = self.get_clock().now()
        self.is_stuck = False
        self.initial_left_wheel_pos = self.left_wheel_pos
        self.initial_right_wheel_pos = self.right_wheel_pos

def main(args=None):
    rclpy.init(args=args)
    locomotion_controller = LocomotionControllerNode()
    try:
        rclpy.spin(locomotion_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            locomotion_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()