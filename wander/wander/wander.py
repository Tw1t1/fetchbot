#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
import random, math

class WanderNode(Node):
    def __init__(self):
        super().__init__('wander')
        self.publisher = self.create_publisher(Heading, 'wander', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second
        self.linear_speed = 0.5 # Adjust as needed

    def timer_callback(self):
        heading = Heading()
        heading.distance = self.linear_speed
        heading.angle = random.uniform(math.pi/2, -math.pi/2)  # Turn between 90 and -90 degrees
        self.publisher.publish(heading)

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
