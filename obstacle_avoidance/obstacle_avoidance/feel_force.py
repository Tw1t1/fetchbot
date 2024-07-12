#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from fetchbot_interfaces.msg import Force


class FeelForceNode(Node):
    def __init__(self):
        super().__init__("feel_force")
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Force, 'force', 10)

    # Callback function for the scan topic
    def scan_callback(self, msg):
        force_vector = self.calculate_force(msg)
        self.publisher.publish(force_vector)

    # Calculate the force vector
    def calculate_force(self, scan):
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        # Filter valid ranges
        valid_indices = np.where((ranges >= scan.range_min) & (ranges <= scan.range_max))
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        # Scale the valid ranges
        scale_ranges = valid_ranges * 2     # Scale by 2, adjust as needed

        # Inverse cube scaling
        inverse_cube_scale_ranges = 1 / (scale_ranges ** 3)

        # Calculate the forces in x and y directions
        forces_x = np.cos(valid_angles) * inverse_cube_scale_ranges
        forces_y = np.sin(valid_angles) * inverse_cube_scale_ranges

        # Calculate the net force
        net_force_x = np.sum(forces_x)
        net_force_y = np.sum(forces_y)

        # Calculate the magnitude and angle of the net force
        magnitude = math.sqrt(net_force_x**2 + net_force_y**2)  # Calculate the magnitude
        angle = math.atan2(net_force_y, net_force_x) # Calculate the angle in radians

        # Create a FeelForce message
        force = Force()
        force.magnitude = magnitude
        force.direction = angle

        return force


def main(args=None):
    rclpy.init(args=args)
    feel_force_node = FeelForceNode()
    rclpy.spin(feel_force_node)
    feel_force_node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()