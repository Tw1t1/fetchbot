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
            10)
        self.joint_states_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Control parameters
        self.max_linear_velocity = 0.4
        self.angular_velocity = 1.0
        self.min_linear_velocity = 0.1
        self.stopping_distance = 0.2
        self.reverse_speed = -0.2

        # State variables
        self.current_heading = None
        self.obstacle_detected = False
        self.last_action = 'none'

        # Timeout handling
        self.last_heading_time = self.get_clock().now()
        self.heading_timeout = 0.5  # Stop if no heading received for 0.5 seconds

    def heading_callback(self, msg):
        self.current_heading = msg
        self.last_heading_time = self.get_clock().now()
        if msg.distance < 0.0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def joint_states_callback(self, msg):
        self.left_wheel_pos = msg.position[0]
        self.right_wheel_pos = msg.position[2]

    def timer_callback(self):
        current_time = self.get_clock().now()
        if (self.current_heading is None or
            (current_time - self.last_heading_time).nanoseconds / 1e9 > self.heading_timeout):
            self.stop_robot()
            return

        twist = Twist()

        if self.obstacle_detected:
            if self.last_action == 'reverse':
                # If we just reversed, prioritize turning
                twist.linear.x = 0.0
                twist.angular.z = self.calculate_desired_angular_velocity()
                self.last_action = 'turn'
            else:
                # Otherwise, reverse
                twist.linear.x = self.reverse_speed
                twist.angular.z = 0.0
                self.last_action = 'reverse'
        else:
            # Normal operation
            twist.linear.x = self.calculate_desired_linear_velocity()
            twist.angular.z = self.calculate_desired_angular_velocity()
            self.last_action = 'normal'
        self.publisher.publish(twist)


    def calculate_desired_linear_velocity(self):
        if self.obstacle_detected:
            return 0.0
        
        if self.current_heading.distance < self.stopping_distance:
            return 0.0
        
        # Adjust velocity based on distance to target
        velocity = min(self.max_linear_velocity, self.current_heading.distance)
        
        # Slow down when turning
        turn_factor = abs(self.current_heading.angle) / math.pi
        velocity *= (1 - 0.5 * turn_factor)
        
        return max(velocity, self.min_linear_velocity)

    def calculate_desired_angular_velocity(self):
        if abs(self.current_heading.angle) < 0.1:  # Small angle threshold
            return 0.0
        
        # Adjust angular velocity based on the angle to turn
        velocity = min(self.angular_velocity, abs(self.current_heading.angle))
        velocity *= math.copysign(1, self.current_heading.angle)
        
        # Increase turning speed when obstacle is detected
        if self.obstacle_detected:
            velocity *= 1.5
        
        return velocity

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.last_action = 'none'


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