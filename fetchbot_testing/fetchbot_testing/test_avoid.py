#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
from geometry_msgs.msg import Twist

class TestAvoidNode(Node):
    def __init__(self):
        super().__init__('test_avoid')
        self.subscription_force = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.test_callback,
            10)
        self.publisher = self.create_publisher(Heading, 'wander', 10)

    def test_callback(self, msg):
        heading = Heading()
        heading.distance = msg.linear.x
        heading.angle = msg.angular.z
        self.publisher.publish(heading)

def main(args=None):
    rclpy.init(args=args)
    test_node = TestAvoidNode()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            test_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
