import os
import json
import cv2
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import BallInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TestBallDetectionNode(Node):
    def __init__(self):
        super().__init__('test_ball_detection')

        self.video_file_name = 'blue_ball_upto_2m_diff_light_condition_640x480.mp4'
        self.publish_rate = 30.0 # Default to 30 Hz

        self.publisher_ = self.create_publisher(Image, '/camera_sensor/image_raw', rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.subscriber_ = self.create_subscription(BallInfo, '/detected_ball', self.count_detection_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.cv_bridge = CvBridge()
        

        self.load_video()
        self.load_ground_truth()

        self.total_detections = 0
        self.total_frames = 0
        

    def load_video(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.video_path = os.path.join(current_dir, '..', 'resource', self.video_file_name)
        
        self.video_capture = cv2.VideoCapture(self.video_path)
        if not self.video_capture.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")

    def load_ground_truth(self):
        # Load the ground truth data from the JSON file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, '..', 'resource', 'video_tags.json')
        with open(json_path, 'r') as f:
            self.ground_truth = json.load(f)

        self.ground_truth_positives = sum(int(value) for value in self.ground_truth.values())


    def timer_callback(self):
        ret, frame = self.video_capture.read()
        
        if ret:
            self.total_frames += 1
            # Convert the frame to ROS Image message
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the image
            self.publisher_.publish(ros_image)
        else:
            self.calculate_success_rate()
            self.get_logger().info("End of video reached. Shutting down the node.")
            self.video_capture.release()
            self.timer.cancel()
            rclpy.shutdown()

    def count_detection_callback(self, ball_info):
        self.total_detections += 1

    def calculate_success_rate(self):
        self.get_logger().info(f"Total frames processed: {self.total_frames}")
        self.get_logger().info(f"Total frames in ground truth: {len(self.ground_truth)}")
        self.get_logger().info(f"Ground truth positive detections: {self.ground_truth_positives}")
        self.get_logger().info(f"Total detections by our system: {self.total_detections}")
        
        if self.ground_truth_positives > 0:
            recall = self.total_detections / self.ground_truth_positives
            self.get_logger().info(f"Recall (Detection Rate): {recall:.2f}")
        else:
            self.get_logger().info("Recall cannot be calculated (no positive samples in ground truth)")
        
        if self.total_frames > 0:
            detection_ratio = self.total_detections / self.total_frames
            self.get_logger().info(f"Detection ratio (Detections per frame): {detection_ratio:.2f}")
        else:
            self.get_logger().info("Detection ratio cannot be calculated (no frames processed)")

    # def timer_callback(self):
    #     ret, frame = self.video_capture.read()
        
    #     if ret:
    #         self.current_frame += 1
    #         # Convert the frame to ROS Image message
    #         ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
    #         # Publish the image
    #         self.publisher_.publish(ros_image)

    #         # Check for false negatives at the end of each frame
    #         if str(self.current_frame) in self.ground_truth and self.current_frame not in self.processed_frames:
    #             if self.ground_truth[str(self.current_frame)] == 1:
    #                 self.false_negatives += 1
    #             self.processed_frames.add(self.current_frame)
            
    #     else:
    #         self.calculate_success_rate()
    #         self.get_logger().info("End of video reached. Shutting down the node.")
    #         self.video_capture.release()
    #         self.timer.cancel()

    # def count_detection_callback(self, ball_info):
    #     if str(self.current_frame) in self.ground_truth:
    #         ground_truth = self.ground_truth[str(self.current_frame)]
            
    #         if ground_truth == 1:
    #             self.true_positives += 1
    #             self.false_negatives -= 1  # Correct a potential false negative
    #         else:
    #             self.false_positives += 1
            
    #         self.processed_frames.add(self.current_frame)

    # def calculate_success_rate(self):
    #     total_frames = self.current_frame
    #     total_detections = sum(int(value) for value in self.ground_truth.values())
        
    #     precision = self.true_positives / (self.true_positives + self.false_positives) if (self.true_positives + self.false_positives) > 0 else 0
    #     recall = self.true_positives / (self.true_positives + self.false_negatives) if (self.true_positives + self.false_negatives) > 0 else 0
    #     f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
        
    #     self.get_logger().info(f"Total frames processed: {total_frames}")
    #     self.get_logger().info(f"Ground truth detections: {total_detections}")
    #     self.get_logger().info(f"True positives: {self.true_positives}")
    #     self.get_logger().info(f"False positives: {self.false_positives}")
    #     self.get_logger().info(f"False negatives: {self.false_negatives}")
    #     self.get_logger().info(f"Precision: {precision:.2f}")
    #     self.get_logger().info(f"Recall: {recall:.2f}")
    #     self.get_logger().info(f"F1 Score: {f1_score:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TestBallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()