import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing

import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Image

import pytest
CAMERA_TOPIC = "camera_sensor/image_raw"

# launch featurs node
def generate_test_description():

    file_path = os.path.dirname(__file__)
    camera_node = launch_ros.actions.Node(
            executable=sys.executable, 
            arguments=[os.path.join(file_path, "..","ball_follow", "camera.py")],
        )

    return (
        launch.LaunchDescription([
            camera_node,
            launch_testing.actions.ReadyToTest(), 
        ]),
        {'camera': camera_node,}
    )

# test node
# Before test you should connnect the usb camera
# and uncomment the follow line in camera.py file
#                   self.get_logger().info(f'Published image id: {img_msg.header.frame_id}')
@pytest.mark.rostest
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


    def test_camera_publish(self, camera, proc_output):
        
        
        msgs_received = []
        
        sub = self.node.create_subscription(
            Image, CAMERA_TOPIC, lambda msg: msgs_received.append(msg), rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        

        try:
            # wait to camera transmit 30 msgs
            end_time = time.time() + 10
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
                
                stamp = f'Published image id: {msg.header.frame_id}'

                proc_output.assertWaitFor(
                    expected_output=stamp, process=camera
                )
        
        finally:
            self.node.destroy_subscription(sub)


    def test_camera_fps(self, camera, proc_output):

        received_images = 0
        total_delay = 0.0
        last_stamp = None
        
        def image_callback(msg):
            nonlocal received_images, total_delay, last_stamp

            stamp_time = Time.from_msg(msg.header.stamp)
            
            if last_stamp is None:
                last_stamp = stamp_time

            delay = (stamp_time - last_stamp).nanoseconds / 1e9
            total_delay += delay
            received_images += 1

            last_stamp = stamp_time
        
        sub = self.node.create_subscription(
            Image, CAMERA_TOPIC, image_callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        try:
            test_duration = 10.0
            end_time = time.time() + test_duration
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=test_duration)

            if received_images > 0:
                average_delay = total_delay / received_images
                rate = 1 / average_delay
                expected_rate = 30.0


                # Assert that the rate is within acceptable bounds

                # self.assertAlmostEqual(rate, 30, delta=3, 
                #                      msg=f"FPS is not close to 30. Measured FPS: {rate:.2f}")
                self.assertGreaterEqual(rate, expected_rate * 0.8,
                    f"Image sending rate too low. Current rate: {rate:.2f} images/second")
                self.assertLessEqual(rate, expected_rate * 1.1,
                    f"Image sending rate too high. Current rate: {rate:.2f} images/second")
            else:
                self.fail("No images received during the test")

        finally:
            self.node.destroy_subscription(sub)

