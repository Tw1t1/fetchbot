import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestFollowBallNode(Node):
    def __init__(self):
        super().__init__('test_ball_follow')
        self.subscription = self.create_subscription(
            String, '/follow_ball/status', self.count_follow_status_callback, 10)
        
        self.follow_count = 0

    def count_follow_status_callback(self, msg):
        pass

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