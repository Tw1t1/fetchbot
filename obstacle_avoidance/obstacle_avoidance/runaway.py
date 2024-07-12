import rclpy
import math
from rclpy.node import Node
from fetchbot_interfaces.msg import Force
from fetchbot_interfaces.msg import Heading


class RunAwayNode(Node):
    def __init__(self):
        super().__init__('runaway')
        self.subscription = self.create_subscription(
            Force,
            'force',
            self.force_callback,
            10)
        self.publisher = self.create_publisher(Heading, '/heading/runaway', 10)
        self.min_force_threshold = 33  # Adjust as needed
        self.max_speed = 3.0  # Adjust as needed

    def force_callback(self, msg):
        # If the force is greater than the threshold, calculate the heading
        if msg.magnitude > self.min_force_threshold:
            heading = self.calculate_heading(msg)
            self.publisher.publish(heading)

    def calculate_heading(self, force):
        heading = Heading()

        # Simple obstacle avoidance: if there's a force, move away from it
        heading.distance = -force.magnitude
        heading.angle = -force.direction
        
        # Normalize the heading to be within the range of max_speed
        if abs(heading.distance) > self.max_speed:
            heading.distance = abs(heading.distance) / heading.distance
        if abs(heading.angle) > self.max_speed:
            heading.angle = abs(heading.angle) / heading.angle
        
        return heading

def main(args=None):
    rclpy.init(args=args)
    runaway = RunAwayNode()
    rclpy.spin(runaway)
    runaway.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()