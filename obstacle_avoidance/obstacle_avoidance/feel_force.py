#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import numpy as np
 
 
class FeelForceNode(Node):
    def __init__(self):
        super().__init__("feel_force")
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Vector3, 'force', 10)

    def scan_callback(self, msg):
        force_vector = self.calculate_force(msg)
        self.publisher.publish(force_vector)

    def calculate_force(self, scan):
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        # Filter valid ranges
        valid_indices = np.where((ranges >= scan.range_min) & (ranges <= scan.range_max))
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        if len(valid_ranges) > 0 and valid_ranges.min() < 0.5:
            self.get_logger().info(f"Valid: {valid_ranges.min()}")

        # Scale the valid ranges
        scale_ranges = valid_ranges * 2     # Scale by 2, adjust as needed

        # Inverse cube scaling
        inverse_cube_scale_ranges = 1 / (scale_ranges ** 3)

        if len(inverse_cube_scale_ranges) > 0:
            self.get_logger().info(f"Inverse: {inverse_cube_scale_ranges.max()}")

        forces_x = np.cos(valid_angles) * inverse_cube_scale_ranges
        forces_y = np.sin(valid_angles) * inverse_cube_scale_ranges

        net_force_x = np.sum(forces_x)
        net_force_y = np.sum(forces_y)

        force_vector = Vector3()
        force_vector.x = net_force_x
        force_vector.y = net_force_y
        force_vector.z = 0.0

        return force_vector


def main(args=None):
    rclpy.init(args=args)
    feel_force_node = FeelForceNode()
    rclpy.spin(feel_force_node)
    feel_force_node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()