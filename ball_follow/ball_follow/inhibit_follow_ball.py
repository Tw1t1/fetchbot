#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # TODO modifie to Heading msg
from std_msgs.msg import Bool # TODO modifie to TBD msg
import time

# worth too look:
# https://answers.ros.org/question/301480/dynamically-resolve-message-type-and-subscribe-to-topic-at-runtime-in-roscpp/

class InhibitorNode(Node):
    
    def __init__(self):
        super().__init__('grab_ball_follow_ball_inhibitore')
        
        # Subscribe to topic_a and topic_b with specified message type
        self.sub_a = self.create_subscription(Twist, '/ball_follow', self.callback_a, 10) # TODO modifie to Heading msg
        self.sub_b = self.create_subscription(Bool, '/TBD', self.callback_b, 10) # TODO modifie to TBD msg


        # creat a publishe topic 
        self.pub_c = self.create_publisher(Twist, 'grab_ball_follow_ball_inhibitore', 10) # TODO modifie to Heading msg

        # Flag to determine if publishing to c is inhibited
        self.inhibit_publish = False
        
        
        self.rcv_timeout_secs = 1.0
        self.lastrcvtime = time.time() - 10000


    def callback_a(self, msg):
        # Publish the message from topic_a to topic_c
        if not self.inhibit_publish:
            self.pub_c.publish(msg) 
        # if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
        



    def callback_b(self, msg):
        self.lastrcvtime = time.time()
        self.inhibit_publish = msg.data

def main(args=None):
    rclpy.init(args=args)
    inhibitor_node = InhibitorNode()
    rclpy.spin(inhibitor_node)
    inhibitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

