import rclpy
import math
from rclpy.node import Node
from fetchbot_interfaces.msg import Force
from fetchbot_interfaces.msg import Heading


class RunAwayNode(Node):
    def __init__(self):
        super().__init__('runaway')
        self.subscription = self.create_subscription(
            Force,
            'force',
            self.force_callback,
            10)
        self.publisher = self.create_publisher(Heading, 'runaway', 10)
        self.linear_speed = 0.5  # Adjust as needed
        self.angular_speed = 3.0    # Adjust as needed
        self.min_force_threshold = 33  # Adjust as needed
        self.back_force_threshold = 100  # Adjust as needed
        self.slow_force_threshold = 300  # Afdjust as needed
        self.stop_force_threshold = 500  # Adjust as needed
        self.back_angle_threshold = math.pi/2  # Adjust as needed
        self.slow_angle_threshold = math.pi/8  # Adjust as needed
        self.stop_angle_threshold = math.pi/24  # Adjust as needed

    def force_callback(self, msg):
        # If the force is greater than the threshold, calculate the heading
        if msg.magnitude > self.min_force_threshold:
            heading = self.calculate_heading(msg)
            self.publisher.publish(heading)

    def calculate_distance(self, distance, force):
        if force.magnitude > self.stop_force_threshold and abs(force.direction) > self.stop_angle_threshold and abs(force.direction) < (math.pi- self.stop_angle_threshold):
            distance = 0.0
        # If the force is greater than the slow force threshold and in the slow zone (1/4 pi - 3/4 pi), slow down
        elif force.magnitude > self.slow_force_threshold and abs(force.direction) > self.slow_angle_threshold and abs(force.direction) < (math.pi- self.slow_angle_threshold):
            distance = distance / 3.0
        # If the force is behind the robot, move backwards
        elif force.magnitude > self.back_force_threshold and abs(force.direction) > self.back_angle_threshold:
            distance = -distance
        return distance


    def calculate_heading(self, force):
        heading = Heading()

        distance = self.calculate_distance(self.linear_speed, force)        

        # Calculate the opposite direction angle
        # opposite_angle = force.direction + math.pi if force.direction < 0 else force.direction - math.pi
        
        heading.distance = distance
        # heading.angle = opposite_angle
        heading.angle = self.angular_speed * (force.direction / math.pi)

        return heading

def main(args=None):
    rclpy.init(args=args)
    runaway = RunAwayNode()
    try:
        rclpy.spin(runaway)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            runaway.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()