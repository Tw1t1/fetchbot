# Copyright 2023 Josh Newans (modifications by Yosef Seada)


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
import time
from math import pi
from enum import Enum


# """
# This module implements a ball follower node for ROS2.
# It tracks a ball, follows it, and publishes its status.
# """

class BallFollowerStatus(Enum):
    STOP = 0
    FOLLOW = 1
    SEARCH = 2
    APPROACH = 3
    WAIT = 4

class FollowBall(Node):

    def __init__(self):

        """
        Initialize the FollowBall node.
        """
        
        super().__init__('follow_ball')
        self.subscription = self.create_subscription(Point, 'orient_home_ball_detection_inhibitore', self.follow_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/follow_ball', 10)
        self.status_publisher = self.create_publisher(String, '/follow_ball/status', 10)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.current_status = BallFollowerStatus.STOP

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)
        self.declare_parameter("search_rotations", 2)  # New parameter for number of rotations


        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        self.search_rotations = self.get_parameter('search_rotations').get_parameter_value().integer_value

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

        # Variables for search behavior
        self.ball_detected = False
        self.search_start_time = None
        self.total_rotation = 0.0

        self.get_logger().info('FollowBall node has been initialized')

    def timer_callback(self):
        
        msg = Twist()
        current_time = time.time()
        
        if current_time - self.lastrcvtime < self.rcv_timeout_secs:
            # Ball is currently detected, follow it
            self.ball_detected = True
            self.current_status = BallFollowerStatus.FOLLOW

            if self.target_dist < self.max_size_thresh:
                msg.linear.x = self.forward_chase_speed
            msg.angular.z = -self.angular_chase_multiplier * self.target_val
            self.search_start_time = None
            self.total_rotation = 0.0

        elif self.ball_detected:
            # Ball was detected before, but now lost. Start or continue search.
            if self.search_start_time is None:
                self.search_start_time = current_time
                self.total_rotation = 0.0

            pi_multiplier = 10 * pi # TODO: 2*pi instead 10*pi is correct but not doing the searc_rotations  with 2 value 
            rotation_needed =  pi_multiplier*self.search_rotations
            time_passed = current_time - self.search_start_time
            self.total_rotation = abs(self.search_angular_speed * time_passed)

            if self.total_rotation < rotation_needed:
                self.current_status = BallFollowerStatus.SEARCH
                msg.angular.z = self.search_angular_speed
            else:
                self.current_status = BallFollowerStatus.STOP
                # self.get_logger().info('Search completed, stopping')
                msg.angular.z = 0.0
                self.ball_detected = False

        else:
            # No ball has been detected yet, or search has completed without finding the ball
            self.current_status = BallFollowerStatus.WAIT
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

        # msg = Twist()
        # if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
        #     self.get_logger().info('Target: {}'.format(self.target_val))
        #     print(self.target_dist)
        #     if (self.target_dist < self.max_size_thresh):
        #         msg.linear.x = self.forward_chase_speed
        #     msg.angular.z = -self.angular_chase_multiplier*self.target_val
        # else:
        #     self.get_logger().info('Target lost')
        #     msg.angular.z = self.search_angular_speed
        # self.publisher_.publish(msg)

    def follow_callback(self, msg):
        """
        Callback for receiving ball position updates.
        """
        try:
            f = self.filter_value
            self.target_val = self.target_val * f + msg.x * (1-f)
            self.target_dist = self.target_dist * f + msg.z * (1-f)
            self.lastrcvtime = time.time()

            self.ball_detected = True
            self.get_logger().info(f'Received Point: ({msg.x}, {msg.y}) | ball size: {msg.z}')
            
        except Exception as e:
            self.get_logger().error(f'Error in listener_callback: {str(e)}')


    def publish_status(self):
        """
        Publish the current status of the ball follower.
        """
        try:
            status_msg = String()
            if self.current_status == BallFollowerStatus.SEARCH:
                status_msg.data = f'SEARCHING, {self.total_rotation / (10*pi):.2f} rotations of {self.search_rotations}'

            elif self.current_status == BallFollowerStatus.FOLLOW:
                status_msg.data = f'FOLLOWING, Target={self.target_val:.2f}, Distance={self.target_dist:.2f}'

            elif self.current_status == BallFollowerStatus.APPROACH:
                status_msg.data = f'APPROACHING, Target={self.target_val:.2f}, Distance={self.target_dist:.2f}'
            
            elif self.current_status == BallFollowerStatus.STOP:
                status_msg.data = f'STOPPED, Distance={self.target_dist:.2f}'

            elif self.current_status == BallFollowerStatus.WAIT:
                status_msg.data = 'WAITING, for initial ball detection'
            else:
                status_msg.data = 'ERROR'

            self.status_publisher.publish(status_msg)
            self.get_logger().info(f'Status: {status_msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error in publish_status: {str(e)}')








def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    follow_ball = FollowBall()
    try:
        rclpy.spin(follow_ball)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        follow_ball.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        follow_ball.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()