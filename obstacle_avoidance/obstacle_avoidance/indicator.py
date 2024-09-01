import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# TODO - update subscription msg type to the correct message type
class IndicatorNode(Node):
    def __init__(self):
        super().__init__('indicator')
        self.subscription = self.create_subscription(
            String,
            'collision',
            self.collision_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'follow_ball/status',
            self.follow_ball_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'orient_home/status',
            self.orient_home_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'low_battery',
            self.low_battery_callback,
            10)
        self.publisher = self.create_publisher(String, 'indicate', 10)
        self.status = {'collision': False}, {'follow_ball': False}, {'orient_home': False}, {'low_battery': False}

    def collision_callback(self, msg):
        self.status['collision'] = True
        self.publish_status()
    
    def follow_ball_callback(self, msg):
        self.status['follow_ball'] = True
        self.publish_status()

    def orient_home_callback(self, msg):
        self.status['orient_home'] = True
        self.publish_status()

    def low_battery_callback(self, msg):
        self.status['low_battery'] = True
        self.publish_status()
    
    def publish_status(self):
        msg = String()
        # combine all status into a single message
        msg.data = ' '.join([key for key, value in self.status.items() if value])
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    indicator = IndicatorNode()
    try:
        rclpy.spin(indicator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            indicator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()