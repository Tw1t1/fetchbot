#!/usr/bin/env python3
import rclpy , math, time
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

        self.significant_force = 0.5 # Adjust as needed
        self.collition_force = 10000 # Adjust as needed

    def force_callback(self, msg):
        self.current_force = msg

    def heading_callback(self, msg):
        self.current_heading = msg
        self.calculate_heading()

    def calculate_heading(self):
        new_heading = Heading()

        if self.current_force.magnitude > self.collition_force and abs(self.current_force.direction) > math.pi/2: 
            # Collision force, and obsticals are in front of the robot
            new_heading.distance = -0.1
            new_heading.angle = self.current_force.direction
    
        elif self.current_force.magnitude > self.significant_force:
            # Significant force, calculate new heading
            self.current_heading.distance = 1.0 if self.current_heading.distance == 0 else self.current_heading.distance
            heading_x = self.current_heading.distance * math.cos(self.current_heading.angle)
            heading_y = self.current_heading.distance * math.sin(self.current_heading.angle)
            
            force_magnitude = max(self.current_force.magnitude, 1.0)
            force_x = force_magnitude * math.cos(self.current_force.direction)
            force_y = force_magnitude * math.sin(self.current_force.direction)

            force_factor = self.calculate_force_factor()
            average_x = force_factor * force_x + (1 - force_factor) * heading_x
            average_y = force_factor * force_y + (1 - force_factor) * heading_y

            new_heading.distance = math.sqrt(average_x**2 + average_y**2)
            new_heading.angle = math.atan2(average_y, average_x)
            self.get_logger().info(f'heading: {self.current_heading.distance:.3f}, {self.current_heading.angle:.3f}')
            self.get_logger().info(f'new_heading: {new_heading.distance:.3f}, {new_heading.angle:.3f}')
            self.get_logger().info(f'force: {self.current_force.magnitude:.3f}, {self.current_force.direction:.3f}, {force_factor:.3f}')
            self.get_logger().info('')

        else:
            # No significant force, maintain current heading
            new_heading = self.current_heading

        self.publisher.publish(new_heading)


    def calculate_force_factor(self):
        # Clamp the input value to a maximum of 3.0
        clamped_value = min(self.current_force.magnitude, 3.0)
        
        # Map the range 0.5-3.0 to 0.1-1.0
        mapped_value = 0.1 + (clamped_value - 0.5) * (0.9 / 2.5)
        
        return mapped_value    


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
