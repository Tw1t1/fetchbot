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
        self.publisher = self.create_publisher(Heading, 'runaway', 10)
        self.min_force_threshold = 33  # Adjust as needed
        self.max_speed = 1.5  # Adjust as needed
        self.max_magnitude = 60  # Adjust as needed

    def force_callback(self, msg):
        # If the force is greater than the threshold, calculate the heading
        if msg.magnitude > self.min_force_threshold:
            heading = self.calculate_heading(msg)
            self.publisher.publish(heading)

    def calculate_heading(self, force):
        heading = Heading()

        # Simple obstacle avoidance: if there's a force, move away from it
        distance = 1.0   # Set the distance to defualt 1
        # Move backwards if the force is in front of the robot
        distance = -distance if abs(force.direction) < math.pi/2 and self.max_magnitude < force.magnitude else distance
        # Calculate the opposite direction angle
        opposite_angle = force.direction + math.pi if force.direction < 0 else force.direction - math.pi
        
        heading.distance = distance
        heading.angle = opposite_angle
        
        return heading

def main(args=None):
    rclpy.init(args=args)
    runaway = RunAwayNode()
    try:
        rclpy.spin(runaway)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            runaway.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()