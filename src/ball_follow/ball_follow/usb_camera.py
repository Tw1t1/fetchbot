#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class USBCamera(Node):

    publisher_name =  "image_publisher"
    topic_name = "image_in"
    
    def __init__(self):

        super().__init__(self.publisher_name)

        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)
        
        self.video_cupture = cv2.VideoCapture(2)
        self.bridge_ = CvBridge()

    
        self.timer_ = self.create_timer(0.0333, self.publish_image)

        self.get_logger().info("Image Publisher Node has been started.")

    def publish_image(self):
        ret, frame = self.video_cupture.read()
        if ret:
            self.get_logger().info("Video frame published")

            image_to_transmit = self.bridge_.cv2_to_imgmsg(frame, 'bgr8')

            self.publisher_.publish(image_to_transmit)
        else:
            self.get_logger().info("Video ended!")


def main(args=None):
    rclpy.init(args=args)
    node = USBCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()