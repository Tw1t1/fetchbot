# Copyright 2023 Josh Newans (modifications by Yosef Seada)


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
import time
from math import pi
from enum import Enum
from ball_follow.state_machine import StateMachine


# """
# This module implements a ball follower node for ROS2.
# It tracks a ball, follows it, and publishes its status.
# """

class BallFollowerStatus(Enum):
    WAITING = 0
    FOLLOWING = 1
    SEARCHING = 2
    APPROACH = 3
    UNREACHABLE = 4
    

class FollowBall(Node):

    def __init__(self):
        super().__init__('follow_ball')

        self.subscription = self.create_subscription(
            Point, '/detected_ball', self.process_ball_detection, 10)
        
        self.publisher_ = self.create_publisher(
            Twist, '/follow_ball', 10)
        
        self.status_publisher = self.create_publisher(
            String, '/follow_ball/status', 10)
        
        self.create_timer(1.0, self.broadcast_current_status)
        self.create_timer(0.1, self.execute_ball_following_cycle)

        self.declare_follow_parameters()
        self.setup_state_machine()
        # Varibles for follow
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

        # Variables for search behavior
        self.ball_detected = False
        self.search_start_time = None
        self.total_rotation = 0.0

        # Varible for unchange ball data
        self.last_ball_data = None
        self.unchanged_start_time = None

        self.get_logger().info('Follow Ball node has been initialized')


    def execute_ball_following_cycle(self):
        """
        Execute the main ball following logic cycle. This method is called periodically.
        """
        msg = Twist()
        current_time = time.time()
        

        # Check if we haven't received ball data for longer than the timeout period
        if current_time - self.lastrcvtime >= self.rcv_timeout_secs:
            if self.ball_detected:
                # Ball was previously detected but now lost
                if self.search_start_time is None:
                    # This is the first time we're starting to search
                    self.search_start_time = current_time
                    self.total_rotation = 0.0
                # Transition to the SEARCHING state to look for the ball
                self.state_machine.transition_to(BallFollowerStatus.SEARCHING)
            else:
                # We haven't detected a ball at all, so just wait
                self.state_machine.transition_to(BallFollowerStatus.WAITING)

        msg = self.state_machine.update()
        
        self.publisher_.publish(msg)
        
        # only for debugging and testing
        self.get_logger().info(f'Heading published, x: {msg.linear.x}, z: {msg.angular.z}')
        # self.get_logger().info(f'Status: {self.state_machine.get_current_state().name}, Heading published, x: {msg.linear.x}, z: {msg.angular.z}')


    def process_ball_detection(self, msg):
        """
        Process incoming ball detection data and update target information.
        """
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1-f)
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()
        self.ball_detected = True

        if self.is_ball_unchanged(msg):
            self.state_machine.transition_to(BallFollowerStatus.UNREACHABLE)
        # elif self.state_machine.get_current_state() == BallFollowerStatus.UNREACHABLE:
        else:
            self.state_machine.transition_to(BallFollowerStatus.FOLLOWING)

        # only for debugging and testing
        self.get_logger().info(f'Received Point: ({msg.x}, {msg.y}) ball size: {msg.z}')


    def is_ball_unchanged(self, current_ball_data):
        """
        Check if the ball's position and size have not changed significantly for a specified duration.
        Position (x, y) is in the range [-1, 1] for each axis and size is in the range [0, 1].
        """
        if self.last_ball_data is None:
            self.last_ball_data = current_ball_data
            self.unchanged_start_time = time.time()
            return False

        # Check x, y and size change
        x_change = abs(current_ball_data.x - self.last_ball_data.x) <= self.position_threshold
        y_change = abs(current_ball_data.y - self.last_ball_data.y) <= self.position_threshold
        size_change = abs(current_ball_data.z - self.last_ball_data.z) <= self.size_threshold

        if (x_change and y_change and size_change) :
            # Ball hasn't changed significantly
            if self.unchanged_start_time is None:
                self.unchanged_start_time = time.time()
            elif time.time() - self.unchanged_start_time > self.unchanged_time_threshold:
                return True
        else:
            # Ball has changed, reset the timer
            self.last_ball_data = current_ball_data
            self.unchanged_start_time = None

        return False

    def broadcast_current_status(self):
        """
        Publish the current status of the ball follower.
        """
        status_msg = String(data=self.state_machine.get_current_state().name)
        self.status_publisher.publish(status_msg)



    def setup_state_machine(self):
        state_actions = {
            BallFollowerStatus.WAITING: self.state_waiting,
            BallFollowerStatus.FOLLOWING: self.state_following,
            BallFollowerStatus.SEARCHING: self.state_searching,
            BallFollowerStatus.UNREACHABLE: self.state_unreachable
        }
        self.state_machine = StateMachine(BallFollowerStatus.WAITING, state_actions)


    def state_waiting(self):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        return msg

    def state_following(self):
        msg = Twist()
        if self.target_dist < self.max_size_thresh:
            msg.linear.x = self.forward_chase_speed
        msg.angular.z = -self.angular_chase_multiplier * self.target_val
        return msg

    def state_searching(self):
        msg = Twist()
        current_time = time.time()
        rotation_needed = 2*pi * self.search_rotations
        time_passed = current_time - self.search_start_time
        self.total_rotation = abs(self.search_angular_speed * time_passed)

        if self.total_rotation < rotation_needed:
            msg.angular.z = self.search_angular_speed
        else:
            self.state_machine.transition_to(BallFollowerStatus.WAITING)
            self.ball_detected = False
        return msg

    def state_unreachable(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return msg
    
    def declare_follow_parameters(self):

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)
        self.declare_parameter("search_rotations", 1)

        self.declare_parameter("position_threshold", 0.05)
        self.declare_parameter("size_threshold", 0.05)
        self.declare_parameter("unchanged_time_threshold", 1.0)

        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        self.search_rotations = self.get_parameter('search_rotations').get_parameter_value().integer_value
        
        self.position_threshold = self.get_parameter('position_threshold').get_parameter_value().double_value
        self.size_threshold = self.get_parameter('size_threshold').get_parameter_value().double_value
        self.unchanged_time_threshold = self.get_parameter('unchanged_time_threshold').get_parameter_value().double_value
   
    def cleanup(self):
        self.get_logger().info('Node shutting down...')


def main(args=None):
    rclpy.init(args=args)
    follow_ball = FollowBall()
    try:
        
        rclpy.spin(follow_ball)
    except KeyboardInterrupt:
        pass
    finally:
        follow_ball.cleanup()
        follow_ball.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()