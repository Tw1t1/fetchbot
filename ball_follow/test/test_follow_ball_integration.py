import os
import sys
import time
import unittest

from ament_index_python import get_package_share_directory
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy

from sensor_msgs.msg import Image
import os
import cv2
from cv_bridge import CvBridge

CAMERA_TOPIC = "camera_sensor/image_raw"

HEADING_TOPIC = '/follow_ball'
FOLLOW_STATUS_TOPIC = '/follow_ball/status'
DETECT_BALL_TOPIC = '/detected_ball'


CAMERA_TOPIC = "/camera_sensor/image_raw"
IMG_TUNING_TOPIC = "/image_tuning"
IMG_OUT_TOPIC = "/image_out"
DETECT_BALL_TOPIC = "/detected_ball"


# launch featurs node
def generate_test_description():
    file_path = os.path.dirname(__file__)

    params_file = os.path.join(get_package_share_directory('ball_follow'),'config','ball_follow_params_robot.yaml')

    ball_detector = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..","ball_follow", "detect_ball.py")],
        parameters=[params_file, {'tuning_mode': False}],
    )
    
    follow_ball = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..","ball_follow", "follow_ball.py")],
        parameters=[params_file, {'tuning_mode': True}],
    )





    return (
        launch.LaunchDescription([
            launch_testing.actions.ReadyToTest(), 
        ]),
        {
        }
    )

# test node

class TestCameraNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # init the ROS context for the test node
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    
    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node("test_camera_link")
    
    def tearDown(self):
        self.node.destroy_node()


    def test_camera_publish(self, camera, proc_output): # proc_output is the file the test framework save the rusult of test
        msgs_received = []

        sub = self.node.create_subscription(
            Image, CAMERA_TOPIC, lambda msg: msgs_received.append(msg), rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        

        try:
            # wait to camera transmit 30 msgs
            end_time = time.time() + 1
            while time.time() < end_time:
                rclpy.spin_once(self.node)

                if len(msgs_received) >= 30: 
                    break

            self.assertGreaterEqual(len(msgs_received), 30, "Did not receive expected number of messages") 

            for i in range(1, len(msgs_received)):
                current_msg = msgs_received[i]
                prev_msg = msgs_received[i-1]

                # Check that timestamps are increasing
                current_time = current_msg.header.stamp.sec + current_msg.header.stamp.nanosec * 1e-9
                prev_time = prev_msg.header.stamp.sec + prev_msg.header.stamp.nanosec * 1e-9
                self.assertGreater(current_time, prev_time,
                                   f"Timestamps not increasing: {prev_msg.header.stamp} to {current_msg.header.stamp}")

            for msg in msgs_received:
                stamp = msg.header.stamp
                stamp = f'published: {stamp}'

                proc_output.assertWaitFor(
                    expected_output=stamp, process=camera
                )
        
        finally:
            self.node.destroy_subscription(sub)

        

    def test_camera_listner(self, camera_listner, proc_output):

        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, 'test_camera_resource', 'test_image.jpg')
        cv_image = cv2.imread(image_path)

        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        pub = self.node.create_publisher(
            Image, CAMERA_TOPIC, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        try:
            for _ in range(10):
                pub.publish(ros_image)

                success = proc_output.waitFor(
                    expected_output=ros_image, process=camera_listner, timeout=1.0
                )
                if success:
                    break
                assert success, 'Waiting for output timed out'
            
        finally:
            self.node.destroy_publisher(pub)


# print("===== Process Information =====")

# for process_name in self.proc_info.process_names():
#     print(f"Process: {process_name}")
#     try:
#         process_info = self.proc_info[process_name]
#         print(f"  Command: {process_info.action.process_details['cmd']}")
#         print(f"  PID: {process_info.action.process_details.get('pid', 'N/A')}")
#         print(f"  Exit Code: {process_info.returncode}")
#     except KeyError:
#         print("  Info not available")
#     print()

# print("\n===== Process Output =====")

# for process_name in self.proc_output.process_names():
#     print(f"Output from {process_name}:")
#     try:
#         process_output = self.proc_output[process_name]
#         for output in process_output:
#             if output.text:
#                 print(f"  {output.text.decode('utf-8').strip()}")
#     except KeyError:
#         print("  No output available")
    # print()
