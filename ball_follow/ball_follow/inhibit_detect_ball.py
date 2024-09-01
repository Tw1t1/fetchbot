#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point # TODO modifie to Ball_Info msg
from std_msgs.msg import Bool # TODO modifie to Bool msg
import time


class InhibitorNode(Node):
    
    def __init__(self):
        super().__init__('ball_detection_inhibitor')
        
        # Subscribe to topic_a and topic_b with specified message type
        self.sub_a = self.create_subscription(Point, '/detected_ball', self.callback_a, 10) # TODO modifie to BallInfo msg
        self.sub_b = self.create_subscription(Bool, '/orient_home/returning_home', self.callback_b, 10) # TODO modifie to BOOL msg


        # creat a publishe topic 
        self.pub_c = self.create_publisher(Point, 'orient_home_detect_ball_inhibitore', 10) # TODO modifie to BallInfo msg

        # Flag to determine if publishing to c is inhibited
        self.inhibit_publish = False 
        self.rcv_timeout_secs = 1.0
        self.lastrcvtime = time.time() - 10000


    def callback_a(self, msg):
        # Publish the message from topic_a to topic_c if not inhibited and within timeout
        current_time = time.time()
        if not self.inhibit_publish: #and (current_time - self.lastrcvtime < self.rcv_timeout_secs):
            self.pub_c.publish(msg)
        



    def callback_b(self, msg):
        self.lastrcvtime = time.time()
        self.inhibit_publish = msg.data
        
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