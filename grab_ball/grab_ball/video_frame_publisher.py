


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera_sensor/image_raw', 10)
        self.cv_bridge = CvBridge()

        # Declare and get parameters
        self.declare_parameter('image_directory', '~/dev_ws/src/fetchbot/ball_follow/test/test_detect_ball_resource')
        self.declare_parameter('publish_rate', 10.0)  # Default to 30 Hz

        self.image_directory = self.get_parameter('image_directory').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Expand user directory if necessary
        self.image_directory = os.path.expanduser(self.image_directory)

        self.get_logger().info(f'Image directory set to: {self.image_directory}')

        if not self.image_directory:
            self.get_logger().error('Image directory parameter is not set')
            return

        if not os.path.exists(self.image_directory):
            self.get_logger().error(f'Image directory does not exist: {self.image_directory}')
            self.get_logger().info(f'Current working directory: {os.getcwd()}')
            return

        if not os.access(self.image_directory, os.R_OK):
            self.get_logger().error(f'No read permissions for directory: {self.image_directory}')
            return

        self.image_files = [f for f in os.listdir(self.image_directory) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
        if not self.image_files:
            self.get_logger().error(f'No image files found in: {self.image_directory}')
            return

        self.get_logger().info(f'Found {len(self.image_files)} image files')

        self.image_files.sort()  # Sort files to ensure consistent order
        self.current_image_index = 0
        self.frame_count = 0
        self.start_time = time.time()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        if not self.image_files:
            self.get_logger().error('No image files to publish')
            return

        image_path = os.path.join(self.image_directory, self.image_files[self.current_image_index])
        frame = cv2.imread(image_path)

        if frame is not None:
            msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher_.publish(msg)
            self.frame_count += 1

            # Calculate actual FPS
            elapsed_time = time.time() - self.start_time
            actual_fps = self.frame_count / elapsed_time

            # Log FPS every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Publishing at {actual_fps:.2f} FPS')

            # Move to the next image, loop back to the start if at the end
            self.current_image_index = (self.current_image_index + 1) % len(self.image_files)
        else:
            self.get_logger().error(f'Failed to read image: {image_path}')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import time
# import os

# class VideoPublisher(Node):
#     def __init__(self):
#         super().__init__('video_publisher')
#         self.publisher_ = self.create_publisher(Image, '/camera_sensor/image_raw', 10)
#         self.timer = self.create_timer(1.0/30.0, self.timer_callback)  # 30 Hz
#         self.cv_bridge = CvBridge()
        
#         # Get the video file path from a parameter
#         self.declare_parameter('video_path', '')
#         video_path = self.get_parameter('video_path').get_parameter_value().string_value
        
#         if not video_path:
#             self.get_logger().error('Video path parameter is not set')
#             return

#         if not os.path.exists(video_path):
#             self.get_logger().error(f'Video file does not exist: {video_path}')
#             return

#         # Try different backends for opening the video
#         backends = [cv2.CAP_GSTREAMER, cv2.CAP_FFMPEG, cv2.CAP_ANY]
#         for backend in backends:
#             self.video = cv2.VideoCapture(video_path, backend)
#             if self.video.isOpened():
#                 self.get_logger().info(f'Successfully opened video with backend {backend}')
#                 break
        
#         if not self.video.isOpened():
#             self.get_logger().error(f'Error opening video file: {video_path}')
#             return

#         self.frame_count = 0
#         self.start_time = time.time()

#     def timer_callback(self):
#         if not hasattr(self, 'video') or not self.video.isOpened():
#             self.get_logger().error('Video is not opened')
#             return

#         ret, frame = self.video.read()
#         if ret:
#             msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
#             self.publisher_.publish(msg)
#             self.frame_count += 1

#             # Calculate actual FPS
#             elapsed_time = time.time() - self.start_time
#             actual_fps = self.frame_count / elapsed_time

#             # Log FPS every 30 frames
#             if self.frame_count % 30 == 0:
#                 self.get_logger().info(f'Publishing at {actual_fps:.2f} FPS')
#         else:
#             self.get_logger().info('End of video file reached')
#             self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset to beginning of video
#             self.frame_count = 0
#             self.start_time = time.time()

# def main(args=None):
#     rclpy.init(args=args)
#     video_publisher = VideoPublisher()
#     rclpy.spin(video_publisher)
#     video_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()