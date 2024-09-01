#!/usr/bin/env python3
import rclpy
import math
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
        new_heading = Heading()

        # Constants for tuning the avoidance behavior
        FORCE_THRESHOLD = 33.0
        MAX_AVOIDANCE_ANGLE = math.pi / 2  # 90 degrees

        if self.current_force.magnitude > FORCE_THRESHOLD:
            # Calculate avoidance angle
            avoidance_angle = min(self.current_force.magnitude / FORCE_THRESHOLD, 1.0) * MAX_AVOIDANCE_ANGLE

            # Determine the direction to turn (opposite of the force direction)
            turn_direction = self.current_force.direction + math.pi
            
            # Calculate the new angle by combining the original heading and the avoidance angle
            new_angle = self.current_heading.angle + avoidance_angle * math.sin(turn_direction - self.current_heading.angle)
            
            # Normalize the new angle to be between -pi and pi
            new_angle = ((new_angle + math.pi) % (2 * math.pi)) - math.pi

            # Reduce speed when avoiding obstacles
            speed_factor = 1.0 - (self.current_force.magnitude / (2 * FORCE_THRESHOLD))
            new_speed = max(0.1, self.current_heading.distance * speed_factor)

            new_heading.angle = new_angle
            new_heading.distance = new_speed
        else:
            # No significant force detected, maintain the original heading
            new_heading = self.current_heading

        self.publisher.publish(new_heading)

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
