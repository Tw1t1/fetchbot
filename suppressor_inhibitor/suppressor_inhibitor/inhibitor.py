from rclpy.node import Node
import time

class InhibitorNode(Node):
    def __init__(self, name, sub1_msg_type, sub1_topic_name, sub2_msg_type, sub2_topic_name, pub_msg_type, pub_topic_name, delay=0.5, function=None):
        super().__init__(name)
        self.subscriber1 = self.create_subscription(
            sub1_msg_type,
            sub1_topic_name,
            self.subscriber1_callback,
            10
        )
        self.subscriber2 = self.create_subscription(
            sub2_msg_type,
            sub2_topic_name,
            self.subscriber2_callback,
            10
        )
        self.publisher = self.create_publisher(pub_msg_type, pub_topic_name, 10)
        self.subscriber1_timer = time.time()
        self.delay = delay
        self.function = function

    def subscriber1_callback(self, msg):
        if self.function is None:   # If function is None, inhibit when message is received
            self.subscriber1_timer = time.time()
        elif self.function(msg):    # If function is not None, inhibit when function returns True
            self.subscriber1_timer = time.time()
        # If function is not None and function returns False, do not inhibit

    def subscriber2_callback(self, msg):
        if time.time() - self.subscriber1_timer > self.delay:
            self.publisher.publish(msg)