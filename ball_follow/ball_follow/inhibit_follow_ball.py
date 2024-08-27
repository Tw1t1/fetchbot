#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # TODO modifie to BallInfo msg
from std_msgs.msg import String 
import time


class InhibitorNode(Node):
    
    def __init__(self):
        super().__init__('follow_ball_inhibitor')
        
        # Subscribe to topic_a and topic_b
        self.sub_a = self.create_subscription(Twist, '/follow_ball', self.callback_a, 10) # TODO modifie to BallInfo msg
        self.sub_b = self.create_subscription(String, '/grab_ball/status', self.callback_b, 10)

        # creat a publishe topic to publish msg from sub_a
        self.pub_c = self.create_publisher(Twist, 'grab_ball_follow_ball_inhibitore', 10) # TODO modifie to BallInfo msg

        # Flag to determine if publishing to c is inhibited
        self.inhibit_publish = False
        self.rcv_timeout_secs = 1.0
        self.lastrcvtime = time.time() - 10000


    def callback_a(self, msg):
        # Publish the message from topic_a to topic_c if not inhibited by topic_b and within timeout
        current_time = time.time()

        # if current_time - self.lastrcvtime < self.rcv_timeout_secs:
        if not self.inhibit_publish:
            self.pub_c.publish(msg)


    def callback_b(self, msg):
        self.lastrcvtime = time.time()
        self.inhibit_publish = msg.data != "WAITING" 
        
    def cleanup(self):
        self.get_logger().info('Node shutting down...')

def main(args=None):
    """
    Main function to initialize and run the node.
    """
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


