import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import BallInfo
from std_msgs.msg import String
import time


class InhibitorNode(Node):
    def __init__(self):
        super().__init__('ball_detection_inhibitor')
        
        # Subscribe to topic_a and topic_b with specified message type
        self.sub_a = self.create_subscription(BallInfo, '/detected_ball', self.callback_a, 10)
        self.sub_b = self.create_subscription(String, '/orient_home/returning_home', self.callback_b, 10)

        # creat a publishe topic 
        self.pub_c = self.create_publisher(BallInfo, 'orient_home_detect_ball_inhibitor', 10)

        # Flag to determine if publishing to c is inhibited
        self.inhibit_publish = False 

    def callback_a(self, msg):
        if not self.inhibit_publish:
            self.pub_c.publish(msg)
        
    def callback_b(self, msg):
        self.lastrcvtime = time.time()
        # self.inhibit_publish = msg.data != "SOME MSG"
        self.inhibit_publish = False
        
    def cleanup(self):
        self.get_logger().info('Node shutting down...')

def main(args=None):
    rclpy.init(args=args)
    inhibitor_node = InhibitorNode()
    try:
        rclpy.spin(inhibitor_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        inhibitor_node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        inhibitor_node.destroy_node()
        inhibitor_node.cleanup()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()