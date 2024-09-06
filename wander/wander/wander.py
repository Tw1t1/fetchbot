#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
import random, math

class WanderNode(Node):
    def __init__(self):
        super().__init__('wander')
        self.publisher = self.create_publisher(Heading, 'wander', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every second
        self.distance = 1.0 # Adjust as needed
        self.last_angle = 0.0
        self.factor = 0.7

    def timer_callback(self):
        heading = Heading()
        random_angle = random.uniform(math.pi/2, -math.pi/2)  # Turn between 90 and -90 degrees
        heading.angle = self.factor * self.last_angle + (1 - self.factor) * random_angle
        heading.distance = self.distance
        self.publisher.publish(heading)
        self.last_angle = heading.angle


def main(args=None):
    rclpy.init(args=args)
    wander_node = WanderNode()
    try:
        rclpy.spin(wander_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            wander_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
