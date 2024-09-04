import os
import sys
import time
import unittest

from ament_index_python import get_package_share_directory
import launch_ros.actions
import launch_testing.actions

import rclpy
from fetchbot_interfaces.msg import Heading, BallInfo
from std_msgs.msg import String
import pytest

from launch import LaunchDescription


HEADING_TOPIC = '/follow_ball'
FOLLOW_STATUS_TOPIC = '/follow_ball/status'
DETECT_BALL_TOPIC = '/detected_ball'


'''
def setUp(self):
    self.proc_output.clear()

proc_output.clear_process_output(process=node_process)
'''
# launch featurs node
@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)

    params_file = os.path.join(get_package_share_directory('ball_follow'),'config','ball_follow_params_robot.yaml')

    follow_ball = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..","ball_follow", "follow_ball.py")],
        parameters=[params_file],
    )

    return (
        LaunchDescription([
            follow_ball,
            launch_testing.actions.ReadyToTest(), 
        ]),
        {
            'follow_ball': follow_ball,
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
        self.node = rclpy.create_node("test_follow_ball")

    def tearDown(self):
        self.node.destroy_node()


    def test_follow_ball_callback(self, follow_ball, proc_output):

        data=[
                (0.1, -0.1, 0.1), 
                (0.5, 0.5, 0.5),
                (1.0, 1.00, 1.0),
                (-0.5,-0.50 ,1.0),
                (-0.2, 0.7, 0.7)
            ]
        ball_info_msgs = self.create_ballinfo_msgs(data)    

        pub = self.get_ball_info_publisher()  
        time.sleep(2.0)
        try:
            
            for ball_info in ball_info_msgs:
                pub.publish(ball_info)

                # for testing uncomment this in detected_ball.py:
                #     self.get_logger().info('Received Point: ({msg.x}, {msg.y}) ball size: {msg.z}')
                keypoint = f'Received Point: ({ball_info.pos_x}, {ball_info.pos_y}) ball size: {ball_info.size}'
                self.node.get_logger().info(keypoint)
                success = proc_output.waitFor(
                    expected_output=keypoint, 
                    process=follow_ball, 
                    timeout = 1.0
                )
                assert success, 'Waiting for output timed out'            
        finally:
            self.node.destroy_publisher(pub)

    def test_publish_heading(self, follow_ball, proc_output):
    
        data={
                (0.1, -0.1, 0.1), 
                (0.5, 0.5, 0.5),
                (1.0, 1.0, 1.0),
                (-0.5,-0.5 ,1.0),
                (-0.2, 0.7, 0.7),
            }
                
        ball_info_msgs = self.create_ballinfo_msgs(data)

        received_heading = []
        pub = self.get_ball_info_publisher()     
        sub = self.get_heading_subscriber(lambda msg: received_heading.append(msg))

        try:

            for i, ball_info in enumerate(ball_info_msgs, 0):

                pub.publish(ball_info)
                
                rclpy.spin_once(self.node, timeout_sec=0.1)

                msg = received_heading[i]
                # for testing uncomment this in follow_ball.py:
                #     self.get_logger().info(f'Heading published, x: {msg.linear.x}, z: {msg.angular.z}')
                self.assertIsNot(0, msg.distance)
                self.assertIsNot(0, msg.angle)
                heading_msg = f'Heading published, x: {msg.distance:.1f}, z: {msg.angle}'
            
            self.assertEqual(len(ball_info_msgs), len(received_heading))
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)


        
    def test_publish_status(self, follow_ball, proc_output):

        received_status = []
  
        sub = self.get_status_subscriber(lambda msg:received_status.append(msg))
        
        try:    
            end_time = time.time() + 5
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

                if len(received_status) > 5: 
                    break
            self.assertGreaterEqual(len(received_status), 5)

            for msg in received_status:
                self.node.get_logger().info(msg.data)
                proc_output.waitFor(
                    expected_output=msg.data, 
                    process=follow_ball, 
                    timeout = 1.0
                )
        finally:
            self.node.destroy_subscription(sub)

    
    
    def test_behavior_state_wait_to_follow(self, follow_ball, proc_output):
        pass
    
    def test_behavior_state_follow_to_search(self, follow_ball, proc_output):
        pass
    
    def test_behavior_state_follow_to_unreachable(self, follow_ball, proc_output):
        pass

    def test_behavior_state_unreachable_to_follow(self, follow_ball, proc_output):
        pass

    def test_behavior_state_search_to_follow(self, follow_ball, proc_output):
        pass

    
    
    # helping methods
    def create_ballinfo_msgs(self, data):
        return [BallInfo(pos_x=float(kp[0]), pos_y=float(kp[1]), size=float(kp[2])) for kp in data]
    def get_ball_info_publisher(self):
        return self.node.create_publisher(BallInfo, DETECT_BALL_TOPIC, 1)
    
    def get_status_subscriber(self, callback):
        return self.node.create_subscription(String, FOLLOW_STATUS_TOPIC, callback, 1)
    
    def get_heading_subscriber(self, callback):
        return self.node.create_subscription(Heading, HEADING_TOPIC, callback, 1)
        