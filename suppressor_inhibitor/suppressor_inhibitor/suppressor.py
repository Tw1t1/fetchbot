from rclpy.node import Node
import time

class SuppressorNode(Node):
    def __init__(self, name, sub1_msg_type, sub1_topic_name, sub2_msg_type, sub2_topic_name, pub_msg_type, pub_topic_name, delay=0.5):
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

    def subscriber1_callback(self, msg):
        self.subscriber1_timer = time.time()
        self.publisher.publish(msg)

    def subscriber2_callback(self, msg):
        if time.time() - self.subscriber1_timer > self.delay:
            self.publisher.publish(msg)