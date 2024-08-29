import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from collections import deque
import numpy as np
from scipy.stats import norm

class ProbabilisticDistanceClassifier:
    def __init__(self, window_size=30):
        self.window_size = window_size
        self.size_buffer = deque(maxlen=window_size)
        self.x_buffer = deque(maxlen=window_size)
        self.y_buffer = deque(maxlen=window_size)

        # Values for alpha = 0.3 (3.0 in the files)
        self.size_mean_8cm, self.size_std_8cm = 0.704672, 0.082195
        self.size_mean_9cm, self.size_std_9cm = 0.539830, 0.159194
        self.x_mean_8cm, self.x_std_8cm = 0.076437, 0.161876
        self.x_mean_9cm, self.x_std_9cm = -0.080689, 0.468453
        self.y_mean_8cm, self.y_std_8cm = 0.873713, 0.074830
        self.y_mean_9cm, self.y_std_9cm = 0.819090, 0.085119

    def calculate_probability(self, observed_value, expected_mean, expected_std):
        return norm.pdf(observed_value, expected_mean, expected_std)

    def classify(self, size, x, y):
        self.size_buffer.append(size)
        self.x_buffer.append(x)
        self.y_buffer.append(y)

        if len(self.size_buffer) < self.window_size:
            return None, 0  # Not enough data yet

        mean_size = np.mean(self.size_buffer)
        mean_x = np.mean(self.x_buffer)
        mean_y = np.mean(self.y_buffer)

        # Calculate probabilities for each feature
        p_size_8cm = self.calculate_probability(mean_size, self.size_mean_8cm, self.size_std_8cm)
        p_size_9cm = self.calculate_probability(mean_size, self.size_mean_9cm, self.size_std_9cm)
        p_x_8cm = self.calculate_probability(mean_x, self.x_mean_8cm, self.x_std_8cm)
        p_x_9cm = self.calculate_probability(mean_x, self.x_mean_9cm, self.x_std_9cm)
        p_y_8cm = self.calculate_probability(mean_y, self.y_mean_8cm, self.y_std_8cm)
        p_y_9cm = self.calculate_probability(mean_y, self.y_mean_9cm, self.y_std_9cm)

        # Calculate overall probability
        p_8cm = p_size_8cm * p_x_8cm * p_y_8cm
        p_9cm = p_size_9cm * p_x_9cm * p_y_9cm

        # Determine estimated distance
        if p_8cm > p_9cm:
            estimated_distance = 8
            confidence = p_8cm / (p_8cm + p_9cm)
        else:
            estimated_distance = 9
            confidence = p_9cm / (p_8cm + p_9cm)

        # Print detailed probabilities for debugging
        print(f"Probabilities - Size: 8cm {p_size_8cm:.4f}, 9cm {p_size_9cm:.4f}")
        print(f"Probabilities - X: 8cm {p_x_8cm:.4f}, 9cm {p_x_9cm:.4f}")
        print(f"Probabilities - Y: 8cm {p_y_8cm:.4f}, 9cm {p_y_9cm:.4f}")
        print(f"Overall Probabilities: 8cm {p_8cm:.4f}, 9cm {p_9cm:.4f}")

        return estimated_distance, confidence

class DistanceClassifierNode(Node):
    def __init__(self):
        super().__init__('distance_classifier_node')
        self.subscription = self.create_subscription(
            Point,
            '/detected_ball',
            self.ball_data_callback,
            10)
        self.classifier = ProbabilisticDistanceClassifier()
        self.message_count = 0
        self.last_time = self.get_clock().now()

    def ball_data_callback(self, msg):
        self.message_count += 1
        
        current_size = msg.z
        x_position = msg.x
        y_position = msg.y
        
        result = self.classifier.classify(current_size, x_position, y_position)
        
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_time).nanoseconds / 1e9
        
        if result[0] is not None:
            estimated_distance, confidence = result
            
            output_msg = (
                f"Messages per second: {self.message_count / time_diff:.2f}, "
                f"Current size: {current_size:.3f}, "
                f"X: {x_position:.3f}, Y: {y_position:.3f}, "
                f"Estimated distance: {estimated_distance} cm, "
                f"Confidence: {confidence:.2f}"
            )
            
            self.get_logger().info(output_msg)
            
            if time_diff >= 1.0:
                self.message_count = 0
                self.last_time = current_time
        else:
            self.get_logger().info(f"Current size: {current_size:.3f}, Collecting more data...")

def main(args=None):
    rclpy.init(args=args)
    distance_classifier_node = DistanceClassifierNode()
    rclpy.spin(distance_classifier_node)
    distance_classifier_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
Last logic before using models for distance:

    # def should_grab_ball(self, x, y, size):
    #     try:
    #         if GrabStatus.GRABBED == self.grab_process_state:
    #             return False
            
    #         position_in_range = self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max
    #         size_in_range = self.ball_size_min <= size <= self.ball_size_max

    #         if position_in_range and size_in_range:
    #             # self.get_logger().info(f'Ball detected at position ({x}, {y}) with size {size}. Should grab.')
    #             return True
    #         else:
    #             # self.get_logger().info(f'Ball detected at position ({x}, {y}) with size {size}. Should not grab.')
    #             return False

    #     except Exception as e:
    #         self.get_logger().error(f'Error in should_grab_ball: {str(e)}')


    in ball_info_callback:
                # if self.should_grab_ball(msg.x, msg.y, msg.z):
            #     self.close_claw()
            #     self.grab_process_state = GrabStatus.CLOSING
            #     self.get_logger().info('Ball detected. Closing claw.')
            # elif self.grab_process_state == GrabStatus.CLOSING and self.current_position > self.position_range_max:
            #     self.open_claw()
            #     self.grab_process_state = GrabStatus.WAITING
            #     self.get_logger().info('Missing grab Ball. Opening claw.')



        self.declare_parameter('ball_size_min', 0.65)
        self.declare_parameter('ball_size_max', 0.9)
        self.declare_parameter('x_min', -0.25)
        self.declare_parameter('x_max', 0.6)
        self.declare_parameter('y_min', 0.4)
        self.declare_parameter('y_max', 0.65)

        # Varibels for ball info to grab
        self.ball_size_min = self.get_parameter('ball_size_min').get_parameter_value().double_value
        self.ball_size_max = self.get_parameter('ball_size_max').get_parameter_value().double_value
        self.x_min = self.get_parameter('x_min').get_parameter_value().double_value
        self.x_max = self.get_parameter('x_max').get_parameter_value().double_value
        self.y_min = self.get_parameter('y_min').get_parameter_value().double_value 
        self.y_max = self.get_parameter('y_max').get_parameter_value().double_value

'''