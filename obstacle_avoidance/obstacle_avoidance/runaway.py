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
        self.significant_force = 0.5 # Adjust as needed
        self.collision_force = 10000 # Adjust as needed

    def force_callback(self, msg):
        # If the force is greater than the significant force, publish the heading
        if msg.magnitude > self.collision_force and abs(msg.direction) > math.pi/2: 
            # Collision force, and obsticals are in front of the robot
            heading = Heading()
            heading.distance = 0.0
            heading.angle = msg.direction
            self.publisher.publish(heading)
        elif msg.magnitude > self.significant_force:
            heading = Heading()
            heading.distance = max(msg.magnitude, 1.0)
            heading.angle = msg.direction
            self.publisher.publish(heading)


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