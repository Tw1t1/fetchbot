#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading, Force

class AvoidNode(Node):
    def __init__(self):
        super().__init__('avoid')
        self.subscription_force = self.create_subscription(
            Force,
            'force',
            self.force_callback,
            10)
        self.subscription_heading = self.create_subscription(
            Heading,
            'orient_home_wander_suppressor',
            self.heading_callback,
            10)
        self.publisher = self.create_publisher(Heading, 'avoid', 10)
        self.current_force = Force()
        self.current_heading = Heading()

    def force_callback(self, msg):
        self.current_force = msg
        self.calculate_heading()

    def heading_callback(self, msg):
        self.current_heading = msg
        self.calculate_heading()

    def calculate_heading(self):
        heading = Heading()

        # TODO implement the avoid algorithm
        # Simple avoid - if there is force bigger then 10 then stop
        if self.current_force.magnitude > 10:
            heading.distance = 0.0
            heading.angle = 0.0
        else:
            heading = self.current_heading

        self.publisher.publish(heading)

def main(args=None):
    rclpy.init(args=args)
    avoid_node = AvoidNode()
    try:
        rclpy.spin(avoid_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            avoid_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
