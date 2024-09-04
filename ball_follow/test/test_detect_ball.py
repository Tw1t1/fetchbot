import os
import sys
import time
import unittest

import cv2

from ament_index_python import get_package_share_directory
import launch_ros.actions
import launch_testing.actions

from cv_bridge import CvBridge
import rclpy
from fetchbot_interfaces.msg import BallInfo
from sensor_msgs.msg import Image
import pytest

from launch import LaunchDescription


CAMERA_TOPIC = "/image_in"
IMG_TUNING_TOPIC = "/image_tuning"
IMG_OUT_TOPIC = "/image_out"
DETECT_BALL_TOPIC = "/detected_ball"

# launch featurs node
@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)

    params_file = os.path.join(get_package_share_directory('ball_follow'),'config','ball_follow_params_robot.yaml')

    ball_detector = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..","ball_follow", "detect_ball.py")],
        # additional_env={'PYTHONUNBUFFERED':'1'}, # to see the result in real time
        parameters=[params_file, {'tuning_mode': False}],
    )

    return (
        LaunchDescription([
            ball_detector,
            launch_testing.actions.ReadyToTest(), 
        ]),
        {
            'ball_detector': ball_detector,
        }
    )

# test node

class TestBallDetectorNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # init the ROS context for the test node
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    
    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node("test_detected_ball")
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.image_dir = os.path.join(current_dir, 'test_detect_ball_resource')


    def tearDown(self):
        self.node.destroy_node()


    # test recived images with ball.
    # test that ball detected and keypoint sent to topic
    def test_detect_ball_callback(self, ball_detector, proc_output):
        
        # kp is BallInfo msg
        received_kps = []
        pub = self.node.create_publisher(Image, CAMERA_TOPIC,1)
            
        sub = self.node.create_subscription(BallInfo, DETECT_BALL_TOPIC,
                                            lambda kp:received_kps.append(kp), 1)
        time.sleep(1.0)
        
        image_files = self.filter_images_by_prefix(os.listdir(self.image_dir), "blue_")
        image_files.sort()

        # for an issue wit first image sent, 
        # the topic of CAMERA_IMAGE ignore the first one msg
        
        pub.publish(self.get_ros_image("without_ball_0.jpg"))
        rclpy.spin_once(self.node, timeout_sec=0.1)

        try:

            for i, image in enumerate(image_files, 0):
                pub.publish(self.get_ros_image(image))
                self.node.get_logger().info(f"Publish {image}")
                rclpy.spin_once(self.node, timeout_sec=0.5)
                
                time.sleep(1.0)
                kp = received_kps[i]
                    
                self.assertTrue(-1.0 <= kp.pos_x <= 1.0)
                self.assertTrue(-1.0 <= kp.pos_y <= 1.0)
                self.assertTrue(0.0  <= kp.size <= 1.0)

                # for testing uncomment this in detected_ball.py:
                #     self.get_logger().info(f"Ball detected: ({point_out.pos_x}, {point_out.pos_y}), {point_out.size}")
                keypoint = f'Ball detected: ({kp.pos_x}, {kp.pos_y}),  {kp.size}'
                success = proc_output.waitFor(
                    expected_output=keypoint, 
                    process=ball_detector, 
                    timeout = 1.0
                )
                assert success, 'Waiting for output timed out'
            
            self.assertGreaterEqual(len(image_files), len(received_kps))
        finally:
            self.clean_average_image()
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)
    

    def test_detect_ball_with_no_ball(self, ball_detector, proc_output):
        received_kps = []

        pub = self.node.create_publisher(Image, CAMERA_TOPIC, 1)    
        sub = self.node.create_subscription(BallInfo, DETECT_BALL_TOPIC,
                                            lambda kp:received_kps.append(kp), 1)
                
        image_files = self.filter_images_by_prefix(os.listdir(self.image_dir), "without_")
        image_files.sort()

        time.sleep(1.0)

        
        try:            
            for image in image_files:
                pub.publish(self.get_ros_image(image))
                self.node.get_logger().info(f"Publish {image}")
                rclpy.spin_once(self.node, timeout_sec=0.3)

                success = proc_output.waitFor(
                    expected_output="Recived image", 
                    process=ball_detector, 
                    timeout = 1.0
                )
                assert success, 'Waiting for output timed out'
        
            self.assertEqual(0, len(received_kps))    
        finally:
            self.clean_average_image()
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)


    def filter_images_by_prefix(self, image_files, prefix):
        valid_extensions = ('.png', '.jpg', '.jpeg', '.bmp')
        
        return [
            f for f in image_files 
            if f.lower().endswith(valid_extensions) and f.startswith(prefix)
            ]
    

    def clean_average_image(self):
        msg = self.get_ros_image("without_ball_0.jpg")
        pub = self.node.create_publisher(Image, CAMERA_TOPIC, 1)
        
        for _ in range(5):
            pub.publish(msg)
            time.sleep(0.1)

        self.node.destroy_publisher(pub)


    def get_ros_image(self, image_name):
        bridge = CvBridge()
        image_path = os.path.join(self.image_dir, image_name)
            
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.node.get_logger().info(f"Failed to read image: {image_name}")
        
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        if ros_image is None:
            self.node.get_logger().info(f"Failed to read image: {image_name}")

        return ros_image