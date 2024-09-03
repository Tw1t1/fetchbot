from rclpy.node import Node
import time

class InhibitorNode(Node):
    def __init__(self, name, sub1_msg_type, sub1_topic_name, sub2_msg_type, sub2_topic_name, pub_msg_type, pub_topic_name, delay=0.5, publish_function=None):
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
        self.subscriber2_msg = None
        self.delay = delay
        self.publish_function = publish_function

    def subscriber1_callback(self, msg):
        self.subscriber1_timer = time.time()

    def subscriber2_callback(self, msg):
        self.subscriber2_msg = msg
        if time.time() - self.subscriber1_timer > self.delay:
            if self.publish_function is not None:
                self.subscriber2_msg = self.publish_function(self.subscriber2_msg)
            self.publisher.publish(self.subscriber2_msg)