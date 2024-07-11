import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
import math

class RunAwayNode(Node):
    def __init__(self):
        super().__init__('run_away')
        self.subscription = self.create_subscription(
            Vector3,
            'force',
            self.force_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        # self.publisher = self.create_publisher(Twist, '/heading/run_away', 10)
        self.min_force_threshold = 33  # Adjust as needed


    def force_callback(self, msg):
        force_magnitude = math.sqrt(msg.x ** 2 + msg.y ** 2)
        
        if force_magnitude > self.min_force_threshold:
            twist_msg = self.calculate_heading(msg)
            self.publisher.publish(twist_msg)

    def calculate_heading(self, force):
        heading = Twist()

        # Simple obstacle avoidance: if there's a force, move away from it
        heading.linear.x = -force.x
        heading.angular.z = -force.y
        # Normalize the heading to be within the range of 1 m/s and 1 rad/s
        if abs(heading.linear.x) > 1:
            heading.linear.x = abs(heading.linear.x) / heading.linear.x
        if abs(heading.angular.z) > 1:
            heading.angular.z = abs(heading.angular.z) / heading.angular.z

        return heading

def main(args=None):
    rclpy.init(args=args)
    run_away = RunAwayNode()
    rclpy.spin(run_away)
    run_away.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()