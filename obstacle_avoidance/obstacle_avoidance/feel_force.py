#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from fetchbot_interfaces.msg import Force, Collision
from rclpy.time import Time

class FeelForceNode(Node):
    def __init__(self):
        super().__init__("feel_force")
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription_collision = self.create_subscription(
            Collision,
            'collision',
            self.collision_callback,
            10)
        self.publisher = self.create_publisher(Force, 'force', 10)
        
        self.latest_scan = None
        self.latest_collision = None
        self.scan_timestamp = None
        self.collision_timestamp = None
        
        # Set timeouts (in seconds)
        self.scan_timeout = 1.0
        self.collision_timeout = 0.5
        
        # Create a timer to periodically check and publish force
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_timestamp = self.get_clock().now()

    def collision_callback(self, msg):
        self.latest_collision = msg
        self.collision_timestamp = self.get_clock().now()

    def timer_callback(self):
        self.calculate_and_publish_force()

    def calculate_and_publish_force(self):
        current_time = self.get_clock().now()
        
        laser_force = None
        if self.latest_scan and self.scan_timestamp:
            if (current_time - self.scan_timestamp).nanoseconds / 1e9 < self.scan_timeout:
                laser_force = self.calculate_laser_force()
            else:
                self.latest_scan = None
                self.scan_timestamp = None
        
        bumper_force = None
        if self.latest_collision and self.collision_timestamp:
            if (current_time - self.collision_timestamp).nanoseconds / 1e9 < self.collision_timeout:
                bumper_force = self.calculate_bumper_force()
            else:
                self.latest_collision = None
                self.collision_timestamp = None

        if laser_force and bumper_force:
            final_force = self.combine_forces(laser_force, bumper_force)
        elif laser_force:
            final_force = laser_force
        elif bumper_force:
            final_force = bumper_force
        else:
            return  # No valid data to publish

        self.publisher.publish(final_force)

    def calculate_free_angle(self, ranges, angles, min_gap_size=0.5, distance_threshold=1.5):
        # Convert inf values to a large number (e.g., twice the distance_threshold)
        ranges[np.isinf(ranges)] = 12.0

        # Find contiguous regions where range > distance_threshold
        free_space = ranges > distance_threshold
        
        # Find the start and end indices of free regions
        region_changes = np.diff(free_space.astype(int))
        start_indices = np.where(region_changes == 1)[0] + 1
        end_indices = np.where(region_changes == -1)[0] + 1
        
        # If the array starts with a free region, add 0 to start_indices
        if free_space[0]:
            start_indices = np.insert(start_indices, 0, 0)
        
        # If the array ends with a free region, add len(free_space) to end_indices
        if free_space[-1]:
            end_indices = np.append(end_indices, len(free_space))
        
        max_gap_size = 0
        max_gap_angle = None
        
        for start, end in zip(start_indices, end_indices):
            gap_size = angles[end - 1] - angles[start]
            if gap_size > max_gap_size and gap_size >= min_gap_size:
                max_gap_size = gap_size
                max_gap_angle = (angles[start] + angles[end - 1]) / 2
        
        return max_gap_angle if max_gap_angle is not None else angles[np.argmax(ranges)]


    def calculate_laser_force(self):
        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        valid_indices = np.where((ranges >= scan.range_min) & (ranges <= scan.range_max))
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        scale_ranges = valid_ranges * 2
        inverse_cube_scale_ranges = 1 / (scale_ranges ** 3)

        forces_x = np.cos(valid_angles) * inverse_cube_scale_ranges
        forces_y = np.sin(valid_angles) * inverse_cube_scale_ranges

        net_force_x = np.sum(forces_x)
        net_force_y = np.sum(forces_y)

        magnitude = math.sqrt(net_force_x**2 + net_force_y**2)
        # angle = math.atan2(net_force_y, net_force_x)
        free_angle = self.calculate_free_angle(ranges, angles)  # Use the free angle instead of the net force angle

        force = Force()
        force.magnitude = magnitude
        force.direction = free_angle

        return force


    def calculate_bumper_force(self):
        force = Force()
        inverse_cube_scale_range = 1 / (self.latest_collision.range ** 3)
        force.magnitude = inverse_cube_scale_range
        force.direction = self.latest_collision.angle
        return force

    def combine_forces(self, laser_force, bumper_force):
        # Convert polar to Cartesian
        laser_x = laser_force.magnitude * math.cos(laser_force.direction)
        laser_y = laser_force.magnitude * math.sin(laser_force.direction)
        bumper_x = bumper_force.magnitude * math.cos(bumper_force.direction)
        bumper_y = bumper_force.magnitude * math.sin(bumper_force.direction)

        # Sum the forces
        total_x = laser_x + bumper_x
        total_y = laser_y + bumper_y

        # Convert back to polar
        combined_force = Force()
        combined_force.magnitude = math.sqrt(total_x**2 + total_y**2)
        combined_force.direction = math.atan2(total_y, total_x)

        return combined_force


def main(args=None):
    rclpy.init(args=args)
    feel_force_node = FeelForceNode()
    try:
        rclpy.spin(feel_force_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            feel_force_node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()