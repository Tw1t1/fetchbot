import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
from geometry_msgs.msg import Twist
import time

class AvoidRunawaySuppressorNode(Node):
    def __init__(self):
        super().__init__('avoid_runaway_suppressor')
        # Create a subscription to the avoid and runaway topics
        self.avoid_subscription = self.create_subscription(
            Heading,
            'heading/avoid',
            self.avoid_callback,
            10
        )
        self.runaway_subscription = self.create_subscription(
            Heading,
            'heading/runaway',
            self.runaway_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        # self.publisher = self.create_publisher(Twist, '/heading/avoid_runaway_suppressor', 10)  # Debugging
        self.last_avoid_time = time.time()
        self.avoid_heading = None
        self.runaway_heading = None

    # Callbacks for the avoid and runaway topics
    def avoid_callback(self, msg):
        self.avoid_heading = msg
        self.last_avoid_time = time.time()
        self.publish_twist(self.avoid_heading)

    # Callback for the runaway topic
    def runaway_callback(self, msg):
        self.runaway_heading = msg
        # Only publish the runaway heading if the avoid heading has not been received in the last 0.5 seconds
        if time.time() - self.last_avoid_time > 0.5:
            self.publish_twist(self.runaway_heading)

    # Convert the heading message to a Twist message and publish it
    def publish_twist(self, heading):
        twist = Twist()
        twist.linear.x = heading.distance
        twist.angular.z = heading.angle
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    avoid_runaway_suppressor_node = AvoidRunawaySuppressorNode()
    rclpy.spin(avoid_runaway_suppressor_node)
    avoid_runaway_suppressor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
