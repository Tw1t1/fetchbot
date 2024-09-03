import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from fetchbot_interfaces.msg import BallInfo
from enum import Enum
import time
import joblib
import os

# Enum to represent the different states of the ball grabbing process
class GrabStatus(Enum):
    WAITING = 0  # Waiting for a ball to be detected
    CLOSING = 1  # Claw is closing to grab the ball
    GRABBED = 2  # Ball has been successfully grabbed

class GrabBall(Node):
    def __init__(self):
        super().__init__('grab_ball')

        self.position_sub = self.create_subscription(Float64, 'position', self.position_callback, 10)
        self.ball_info_sub = self.create_subscription(BallInfo, '/detected_ball', self.ball_info_callback, 10)
        self.grab_ball_status_pub = self.create_publisher(String, 'grab_ball/status', 10)
        self.claw_cmd_pub = self.create_publisher(String, 'claw_cmd', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        # Load machine learning models for ball detection and distance estimation
        self.load_models()
        self.initialize_parameters()
        self.initialize_variables()
        
        self.get_logger().info('Waiting for ball to grab ... ')

    def initialize_parameters(self):
        self.declare_parameter('position_change_threshold', 2.0) 
        self.declare_parameter('position_range_min', 60.0)
        self.declare_parameter('position_range_max', 80.0)
        self.position_change_threshold = self.get_parameter('position_change_threshold').get_parameter_value().double_value
        self.position_range_min = self.get_parameter('position_range_min').get_parameter_value().double_value
        self.position_range_max = self.get_parameter('position_range_max').get_parameter_value().double_value

    def initialize_variables(self):
        # Set up internal variables for tracking grab status and claw position
        self.rcv_timeout_secs = 1.5
        self.lastrcvtime = time.time() - 10000
        self.current_status = GrabStatus.WAITING

        # Variables for grab detection
        self.current_position = -1.0
        self.previous_position = None
        self.unchanged_position_count = 0
        self.stable_position_count_threshold = 3


    def load_models(self):
        # Load pre-trained machine learning models for ball detection and distance estimation
        file_path = os.path.dirname(__file__)
        graspable_model_path = os.path.join(file_path, '..', 'distance_models', 'graspable_classifier.joblib')
        dist_model_path = os.path.join(file_path, '..' , 'distance_models', 'rf_dist_model_0.3m_calib_graspel.joblib')

        self.binary_model = joblib.load(graspable_model_path)
        self.rf_model = joblib.load(dist_model_path)['model']

    def timer_callback(self):
        # This method is called periodically to publish current status of grab ball
        # and check if need to releas the ball
        current_time = time.time()

        if self.current_status == GrabStatus.GRABBED and \
            (current_time - self.lastrcvtime) > self.rcv_timeout_secs: 
            self.open_claw()
            self.current_status = GrabStatus.WAITING
        
        # Publish the current grab status
        status = String()
        status.data = self.current_status.name
        self.grab_ball_status_pub.publish(status)

    def position_callback(self, msg):
        try:
            self.current_position = round(msg.data)

            # If we're waiting for a ball, open the claw. just for ensure the claw always open
            if self.current_status == GrabStatus.WAITING and self.current_position > 10.0:
                self.open_claw()

            # If we've detected a ball grab, stop the claw
            if self.is_ball_grabbed():
                self.stop_claw()
                self.current_status = GrabStatus.GRABBED
        except Exception as e:
            self.get_logger().error(f'Error in position_callback: {str(e)}')

    def should_grab_ball(self, msg):
        # Determine if we should attempt to grab the ball based on its position and our current state
        if GrabStatus.GRABBED == self.current_status:
            return False
        
        estimated_dist, is_grabable = self.predict_distance(msg)

        return is_grabable and estimated_dist <= 7.0

    def ball_info_callback(self, msg):
        self.lastrcvtime = time.time()
        
        try:
            # If we should grab the ball, close it
            if self.should_grab_ball(msg):
                self.close_claw()
                self.current_status = GrabStatus.CLOSING
            # If we're closing but the position is out of range, open the claw (missed grab)
            elif self.current_status == GrabStatus.CLOSING and self.current_position > self.position_range_max:
                self.open_claw()
                self.current_status = GrabStatus.WAITING
        except Exception as e:
            self.get_logger().error(f'Error in ball_info_callback: {str(e)}')

    def is_ball_grabbed(self):
        # Determine if the ball has been successfully grabbed
        if self.current_status != GrabStatus.CLOSING:
            return False

        is_position_in_grab_range = self.position_range_min <= self.current_position <= self.position_range_max

        if not is_position_in_grab_range:
            self.reset_grab_detection()
            return False

        if self.previous_position is None:
            self.previous_position = self.current_position
            return False

        position_change = round(abs(self.current_position - self.previous_position))
        is_position_stable = position_change <= self.position_change_threshold

        if is_position_stable:
            self.unchanged_position_count += 1
            is_grab_detected = self.unchanged_position_count >= self.stable_position_count_threshold
            if is_grab_detected:
                return True
        else:
            self.reset_grab_detection()

        self.previous_position = self.current_position
        return False

    def reset_grab_detection(self):
        # Reset variables used for grab detection
        self.unchanged_position_count = 0
        self.previous_position = None

    def predict_distance(self, ball_info):
        # Use the pre-trained models to predict the distance to the ball and if it's graspable
        features = np.array([[ball_info.size, ball_info.pos_x, ball_info.pos_y]])

        estimated_distance = self.rf_model.predict(features)[0]
        is_graspable = self.binary_model.predict(features)[0]

        return estimated_distance, is_graspable

    def close_claw(self):
        self._publish_claw_command('close')

    def stop_claw(self):
        self._publish_claw_command('stop')

    def open_claw(self):
        self._publish_claw_command('open')

    def _publish_claw_command(self, command):
        # Helper method to publish claw commands
        claw_cmd = String()
        claw_cmd.data = command
        self.claw_cmd_pub.publish(claw_cmd)

def main(args=None):
    rclpy.init(args=args)
    grab_ball_node = GrabBall()
    
    try:
        rclpy.spin(grab_ball_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        grab_ball_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()