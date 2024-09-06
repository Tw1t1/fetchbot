#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from fetchbot_interfaces.msg import Force, Collision

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

        self.collision_force_x = 0.0
        self.collision_force_y = 0.0
        self.last_collision_time = 0.0
        self.collision_duration = 0.5  # Collision force relevance duration in seconds
        self.power_factor = 3  # Adjust as needed
        self.range_factor = 0.75  # Adjust as needed
        self.divisor = 650  # Adjust as needed
        self.collision_range_treshold = 0.23
        self.collision_magnitude = 100000


    def scan_callback(self, msg):
        # Calculate force based on LaserScan data
        force_x = 0.0
        force_y = 0.0
        
        for i, range_value in enumerate(msg.ranges):
            if range_value < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                # Calculate force contribution (inversely proportional to distance squared)
                if range_value <= self.collision_range_treshold:
                    force_magnitude = self.collision_magnitude
                    self.get_logger().info("Collision")
                else:
                    force_magnitude = max(range_value, msg.range_min) / self.range_factor
                    force_magnitude = 1.0 / force_magnitude**self.power_factor
                    force_magnitude /= self.divisor
                force_x += force_magnitude * math.cos(angle)
                force_y += force_magnitude * math.sin(angle)

        # Check if collision force is still relevant
        current_time = time.time()
        if current_time - self.last_collision_time <= self.collision_duration:
            self.get_logger().info("Collision")
            # Combine LiDAR force with collision force
            total_force_x = force_x + self.collision_force_x 
            total_force_y = force_y + self.collision_force_y
        else:
            # Use only LiDAR force if collision is too old
            total_force_x = force_x
            total_force_y = force_y

        # Invert force direction
        total_force_x = -total_force_x
        total_force_y = -total_force_y
        
        # Calculate magnitude and direction
        magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
        direction = math.atan2(total_force_y, total_force_x)

        # Create and publish the Force message
        force_msg = Force()
        force_msg.magnitude = magnitude
        force_msg.direction = direction
        self.publisher.publish(force_msg)

    def collision_callback(self, msg):
        # Calculate force based on collision data
        self.collision_force_x = self.collision_magnitude * math.cos(msg.angle)
        self.collision_force_y = self.collision_magnitude * math.sin(msg.angle)
        self.last_collision_time = time.time()


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