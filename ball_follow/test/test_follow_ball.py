import os
import sys
import time
import unittest

from ament_index_python import get_package_share_directory
import launch_ros.actions
import launch_testing.actions

import rclpy
from geometry_msgs.msg import Point, Twist
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
        parameters=[params_file, {'tuning_mode': True}],
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
                (1.0, 1.0, 1.0),
                (-0.5,-0.5 ,1.0),
                (-0.2, 0.7, 0.07)
            ]
        
        point_msgs = self.create_points(data)    

        pub = self.get_ball_info_publisher()  
        time.sleep(1.0)
        try:
            
            for i, msg in enumerate(point_msgs, 0):

                pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=0.1)


                # for testing uncomment this in detected_ball.py:
                #     self.get_logger().info('Received Point: ({msg.x}, {msg.y}) ball size: {msg.z}')
                keypoint = f'Received Point: ({data[i][0]}, {data[i][1]}) ball size: {data[i][2]}'
                self.node.get_logger().info(keypoint)
                success = proc_output.waitFor(
                    expected_output=keypoint, 
                    process=follow_ball, 
                    timeout = 1.0
                )
                assert success, 'Waiting for output timed out'            
        finally:
            self.node.destroy_publisher(pub)

    # heading = Twist
    def test_publish_heading(self, follow_ball, proc_output):
    
        data={
                (0.1, -0.1, 0.1), 
                (0.5, 0.5, 0.5),
                (1.0, 1.0, 1.0),
                (-0.5,-0.5 ,1.0),
                (-0.2, 0.7, 0.07),
            }
                
        point_msgs = self.create_points(data)

        received_heading = []
        pub = self.get_ball_info_publisher()     
        sub = self.get_heading_subscriber(lambda msg: received_heading.append(msg))

        try:

            for i, point in enumerate(point_msgs, 0):

                pub.publish(point)
                
                self.node.get_logger().info(f"Publish point: {point.x},{point.y},{point.z}")
                rclpy.spin_once(self.node, timeout_sec=0.3)

                msg = received_heading[i]
                # for testing uncomment this in follow_ball.py:
                #     self.get_logger().info(f'Heading published, x: {msg.linear.x}, z: {msg.angular.z}')
                
                heading_msg = f'Heading published, x: {msg.linear.x}, z: {msg.angular.z}'
                success = proc_output.waitFor(
                    expected_output=heading_msg, 
                    process=follow_ball, 
                    timeout = 1.0
                )
                assert success, 'Waiting for output timed out'
            
            self.assertEqual(len(point_msgs), len(received_heading))
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



        '''
    STOPPED = 0
    FOLLOWING = 1
    SEARCHING = 2
    APPROACH = 3
    WAITING = 4
    APPROACHING = 5
    UNREACABLE = 6
        '''

    def test_behavior_state_stop_to_wait(self, follow_ball, proc_output):

        data=[
                (0.1, -0.1, 0.1), 
                (0.5, 0.5, 0.5),
                (1.0, 1.0, 1.0),
                (-0.5,-0.5 ,1.0),
                (-0.2, 0.7, 0.07)
            ]
        
        received_heading = []
        received_status = []

        point_msgs = self.create_points(data)    

        ball_info_pub = self.get_ball_info_publisher()
        status_subscriber = self.get_status_subscriber(lambda msg:received_status.append(msg))
        heading_sub = self.get_heading_subscriber(lambda msg: received_heading.append(msg))
        time.sleep(1.0)
    
        try:
            pass
        finally:
            self.node.destroy_publisher(ball_info_pub)
            self.node.destroy_subscription(heading_sub)
            self.node.destroy_subscription(status_subscriber)
    
    def test_behavior_state_stop_to_follow(self, follow_ball, proc_output):
        pass
    
    def test_behavior_state_wait_to_follow(self, follow_ball, proc_output):
        pass
    
    def test_behavior_state_follow_to_search(self, follow_ball, proc_output):
        pass
    
    def test_behavior_state_follow_to_unreachable(self, follow_ball, proc_output):
        pass

    def test_behavior_state_search_to_follow(self, follow_ball, proc_output):
        pass

    def test_behavior_state_follow_to_stopp(self, follow_ball, proc_output):
        pass
    
    # helping methods
    def create_points(self, data):
        return [Point(x=float(kp[0]), y=float(kp[1]), z=float(kp[2])) for kp in data]
    def get_ball_info_publisher(self):
        return self.node.create_publisher(Point, DETECT_BALL_TOPIC, 1)
    
    def get_status_subscriber(self, callback):
        return self.node.create_subscription(String, FOLLOW_STATUS_TOPIC, callback, 1)
    
    def get_heading_subscriber(self, callback):
        return self.node.create_subscription(Twist, HEADING_TOPIC, callback, 1)
        