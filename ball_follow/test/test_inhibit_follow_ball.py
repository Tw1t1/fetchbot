import os
import sys
import time
import unittest

import launch_ros.actions
import launch_testing.actions

import rclpy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
import pytest

from launch import LaunchDescription


HEADING_TOPIC = '/follow_ball'


# launch featurs node
@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)


    return (
        LaunchDescription([
            launch_testing.actions.ReadyToTest(), 
        ]),
        {
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
        self.node = rclpy.create_node("test_follow_ball_inhibit")

    def tearDown(self):
        self.node.destroy_node()