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

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import Point

# from collections import deque
# import numpy as np
# import time
# from datetime import datetime

# class DynamicDistanceClassifier:
#     def __init__(self, alpha, window_size=30):
#         self.alpha = alpha
#         self.window_size = window_size
#         self.size_buffer = deque(maxlen=window_size)
#         self.x_buffer = deque(maxlen=window_size)
#         self.y_buffer = deque(maxlen=window_size)
        
#         # Expected values remain the same as in the original script
#         self.expected_values = {
#             8: {
#                 'size': {0.1: 0.688705, 0.2: 0.701822, 0.3: 0.704672, 0.4: 0.708014, 0.5: 0.708267, 0.6: 0.711790, 0.7: 0.710836, 0.8: 0.710879, 0.9: 0.714157},
#                 'cv': {0.1: 0.117135, 0.2: 0.114147, 0.3: 0.116643, 0.4: 0.120681, 0.5: 0.122973, 0.6: 0.121735, 0.7: 0.126602, 0.8: 0.128288, 0.9: 0.126662},
#                 'x_std': {0.1: 0.145243, 0.2: 0.154981, 0.3: 0.161876, 0.4: 0.164490, 0.5: 0.166822, 0.6: 0.167296, 0.7: 0.169115, 0.8: 0.170220, 0.9: 0.168472},
#                 'y_std': {0.1: 0.087532, 0.2: 0.074305, 0.3: 0.074830, 0.4: 0.078291, 0.5: 0.080865, 0.6: 0.083221, 0.7: 0.083042, 0.8: 0.084985, 0.9: 0.084504}
#             },
#             9: {
#                 'size': {0.1: 0.508822, 0.2: 0.531683, 0.3: 0.539830, 0.4: 0.543226, 0.5: 0.546073, 0.6: 0.547155, 0.7: 0.549605, 0.8: 0.550159, 0.9: 0.550359},
#                 'cv': {0.1: 0.278809, 0.2: 0.292215, 0.3: 0.294897, 0.4: 0.296852, 0.5: 0.295530, 0.6: 0.297002, 0.7: 0.293012, 0.8: 0.297675, 0.9: 0.295531},
#                 'x_std': {0.1: 0.452743, 0.2: 0.465381, 0.3: 0.468453, 0.4: 0.469418, 0.5: 0.468999, 0.6: 0.469184, 0.7: 0.468620, 0.8: 0.468890, 0.9: 0.469753},
#                 'y_std': {0.1: 0.109182, 0.2: 0.092029, 0.3: 0.085119, 0.4: 0.085297, 0.5: 0.084895, 0.6: 0.083512, 0.7: 0.086305, 0.8: 0.085765, 0.9: 0.086248}
#             }
#         }
        
#         # Calculate expected values based on alpha
#         self.expected_mean_8cm = self._interpolate_value(alpha, self.expected_values[8]['size'])
#         self.expected_mean_9cm = self._interpolate_value(alpha, self.expected_values[9]['size'])
#         self.expected_cv_8cm = self._interpolate_value(alpha, self.expected_values[8]['cv'])
#         self.expected_cv_9cm = self._interpolate_value(alpha, self.expected_values[9]['cv'])
#         self.expected_x_std_8cm = self._interpolate_value(alpha, self.expected_values[8]['x_std'])
#         self.expected_x_std_9cm = self._interpolate_value(alpha, self.expected_values[9]['x_std'])
#         self.expected_y_std_8cm = self._interpolate_value(alpha, self.expected_values[8]['y_std'])
#         self.expected_y_std_9cm = self._interpolate_value(alpha, self.expected_values[9]['y_std'])

#     def _interpolate_value(self, alpha, value_dict):
#         """
#         Interpolate value for given alpha.
#         """
#         alphas = sorted(value_dict.keys())
#         values = [value_dict[a] for a in alphas]
#         return np.interp(alpha, alphas, values)

#     def update_and_classify(self, current_size, x_position, y_position):
#         self.size_buffer.append(current_size)
#         self.x_buffer.append(x_position)
#         self.y_buffer.append(y_position)

#         if len(self.size_buffer) < self.window_size:
#             return None, 0, np.mean(self.size_buffer), 0  # Not enough data yet

#         mean_size = np.mean(self.size_buffer)
#         cv = np.std(self.size_buffer) / mean_size if mean_size != 0 else 0
#         x_std = np.std(self.x_buffer)
#         y_std = np.std(self.y_buffer)

#         distance_score_8cm = 0
#         distance_score_9cm = 0
#         confidence = 0

#         # Compare mean size to expected means
#         size_diff_8cm = abs(mean_size - self.expected_mean_8cm)
#         size_diff_9cm = abs(mean_size - self.expected_mean_9cm)
#         if size_diff_8cm < size_diff_9cm:
#             distance_score_8cm += 1
#             confidence += 0.3
#         else:
#             distance_score_9cm += 1
#             confidence += 0.3

#         # Compare CV to expected CVs
#         cv_diff_8cm = abs(cv - self.expected_cv_8cm)
#         cv_diff_9cm = abs(cv - self.expected_cv_9cm)
#         if cv_diff_8cm < cv_diff_9cm:
#             distance_score_8cm += 1
#             confidence += 0.2
#         else:
#             distance_score_9cm += 1
#             confidence += 0.2

#         # Compare X standard deviation
#         x_std_diff_8cm = abs(x_std - self.expected_x_std_8cm)
#         x_std_diff_9cm = abs(x_std - self.expected_x_std_9cm)
#         if x_std_diff_8cm < x_std_diff_9cm:
#             distance_score_8cm += 0.5
#             confidence += 0.1
#         else:
#             distance_score_9cm += 0.5
#             confidence += 0.1

#         # Compare Y standard deviation
#         y_std_diff_8cm = abs(y_std - self.expected_y_std_8cm)
#         y_std_diff_9cm = abs(y_std - self.expected_y_std_9cm)
#         if y_std_diff_8cm < y_std_diff_9cm:
#             distance_score_8cm += 0.5
#             confidence += 0.1
#         else:
#             distance_score_9cm += 0.5
#             confidence += 0.1

#         # Determine distance based on scores
#         if abs(distance_score_8cm - distance_score_9cm) < 0.5:
#             estimated_distance = "Uncertain"
#         elif distance_score_8cm > distance_score_9cm:
#             estimated_distance = 8
#         else:
#             estimated_distance = 9

#         # Adjust confidence based on score difference
#         score_diff = abs(distance_score_8cm - distance_score_9cm)
#         confidence *= min(score_diff / 3, 1.0)  # Scale confidence by score difference, max 1.0

#         return estimated_distance, confidence, mean_size, cv

# class DistanceClassifierNode(Node):
#     def __init__(self):
#         super().__init__('distance_classifier_node')
#         self.subscription = self.create_subscription(
#             Point,
#             '/detected_ball',
#             self.ball_data_callback,
#             10)
#         self.classifier = DynamicDistanceClassifier(alpha=0.5)
#         self.message_count = 0
#         self.last_time = time.time()

#     def ball_data_callback(self, msg):
#         self.message_count += 1
        
#         current_size = msg.z
#         x_position = msg.x
#         y_position = msg.y
        
#         result = self.classifier.update_and_classify(current_size, x_position, y_position)
        
#         current_time = time.time()
#         time_diff = current_time - self.last_time
        
#         if result[0] is not None:
#             estimated_distance, confidence, mean_size, cv = result
            
#             output_msg = (
#                 f"Msgs per sec: {self.message_count / time_diff:.2f}, "
#                 f"Current size: {current_size:.3f}, "
#                 f"X: {x_position:.3f}, Y: {y_position:.3f}, "
#                 f"Mean size: {mean_size:.3f}, "
#                 f"CV: {cv:.3f}, "
#                 f"Estimated distance: {estimated_distance} cm, "
#                 f"Confidence: {confidence:.2f}"
#             )
            
#             self.get_logger().info(output_msg)
            
#             if time_diff >= 1.0:
#                 self.message_count = 0
#                 self.last_time = current_time
#         else:
#             self.get_logger().info(f"Current size: {current_size:.3f}, Collecting more data...")

# def main(args=None):
#     rclpy.init(args=args)
#     distance_classifier_node = DistanceClassifierNode()
#     rclpy.spin(distance_classifier_node)
#     distance_classifier_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()