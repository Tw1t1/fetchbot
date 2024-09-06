# Copyright 2023 Josh Newans (modifications by Yosef Seada)


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from math import pi
from enum import Enum
from ball_follow.state_machine import StateMachine
from fetchbot_interfaces.msg import Heading, BallInfo

# """
# This module implements a ball follower node for ROS2.
# It tracks a ball, follows it, and publishes its status.
# """

class FollowerStatus(Enum):
    WAITING = 0
    FOLLOWING = 1
    SEARCHING = 2
    UNREACHABLE = 3
    ARRIVED = 4
    

class FollowBall(Node):
    def __init__(self):
        super().__init__('follow_ball')

        self.subscription = self.create_subscription(
            BallInfo, '/detected_ball', self.process_ball_detection, 10)
        
        self.publisher_ = self.create_publisher(
            Heading, '/follow_ball', 10)
                        
        self.status_publisher = self.create_publisher(
            String, '/follow_ball/status', 10)

        self.follow_rate = 1.0/10.0 #  frequency 10 Hz        
        self.create_timer(self.follow_rate, self.broadcast_current_status)
        
        self.create_timer(self.follow_rate, self.execute_ball_following_cycle)

        self.declare_follow_parameters()
        self.setup_state_machine()
        self.declare_follow_variable()

        self.get_logger().info('Follow Ball node has been initialized')


    def execute_ball_following_cycle(self):
        """
        Execute the main ball following logic cycle. This method is called periodically.
        """
        current_time = time.time()
        
        # Check if we haven't received ball data for longer than the timeout period
        if current_time - self.lastrcvtime >= self.rcv_timeout_secs:
            if self.ball_detected:
                # Ball was previously detected but now lost
                # Transition to the SEARCHING state to look for the ball
                self.state_machine.transition_to(FollowerStatus.SEARCHING)
            else:
                # We haven't detected a ball at all, so just wait
                self.state_machine.transition_to(FollowerStatus.WAITING)

        self.state_machine.update()
        


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
        x_change = abs(current_ball_data.pos_x - self.last_ball_data.pos_x) <= self.position_threshold
        y_change = abs(current_ball_data.pos_y - self.last_ball_data.pos_y) <= self.position_threshold
        size_change = abs(current_ball_data.size - self.last_ball_data.size) <= self.size_threshold

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


    def process_ball_detection(self, msg):
        """
        Process incoming ball detection data and update target information.
        """
        f = self.filter_value
        self.target_val = self.target_val * f + msg.pos_x * (1-f)
        self.target_dist = self.target_dist * f + msg.size * (1-f)
        self.lastrcvtime = time.time()
        self.ball_detected = True
        
        if self.is_ball_unchanged(msg):
            self.state_machine.transition_to(FollowerStatus.UNREACHABLE)
        else:
            self.state_machine.transition_to(FollowerStatus.FOLLOWING)

        # only for debugging and testing
        # self.get_logger().info(f'Received Point: ({msg.pos_x:.1f}, {msg.pos_y:.1f}) ball size: {msg.size:.1f}')


    def on_following(self):
        msg = Heading()
        
        if self.target_dist < self.max_size_thresh:
            msg.distance = self.forward_chase_speed
        msg.angle = -self.angular_chase_multiplier * self.target_val

        self.get_logger().info(f'target_val: {self.target_val:.3f}, msg.angle: {msg.angle:.3f}')
        self.publisher_.publish(msg)
        
        self.searching_count = 0 
        # only for debugging and testing
        # self.get_logger().info(f'Heading published, x: {msg.distance}, z: {msg.angle}')


    def on_waiting(self):
        # do nothing
        pass

    # based on if (1.0 / turn_frequency) * count * angular_velocity > turn_angle
    def on_searching(self):
        
        msg = Heading()
        angular_velocity = -self.search_angular_speed * (self.target_val/abs(self.target_val))
        msg.angle = angular_velocity
        
        
        turn_angle = 2 * pi * self.search_rotations * 2.3
        current_angle = self.follow_rate * self.searching_count * abs(angular_velocity)
        
        self.searching_count += 1
        if current_angle > turn_angle:
            self.ball_detected = False
            self.searching_count = 0
            msg.angle = 0.0  # Stop rotating

        self.publisher_.publish(msg)
        # only for debugging and testing
        # self.get_logger().info(f'Heading published, x: {msg.distance}, z: {msg.angle}')


    def on_unreachable(self):
        # right now do nothing, in futer may can upload some specifiec behavior
        pass


    def declare_follow_variable(self):
        # Variables for follow
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

        # Variables for search behavior
        self.ball_detected = False
        self.searching_count = 0
        
        # Variables for unchange ball data
        self.last_ball_data = None
        self.unchanged_start_time = None


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

    def broadcast_current_status(self):
        """
        Publish the current status of the ball follower.
        """
        status_msg = String(data=self.state_machine.get_current_state().name)
        self.status_publisher.publish(status_msg)

    def setup_state_machine(self):
        state_actions = {
            FollowerStatus.WAITING: self.on_waiting,
            FollowerStatus.FOLLOWING: self.on_following,
            FollowerStatus.SEARCHING: self.on_searching,
            FollowerStatus.UNREACHABLE: self.on_unreachable
        }
        self.state_machine = StateMachine(FollowerStatus.WAITING, state_actions)


def main(args=None):
    rclpy.init(args=args)
    follow_ball = FollowBall()
    try:
        
        rclpy.spin(follow_ball)
    except KeyboardInterrupt:
        pass
    finally:
        follow_ball.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()