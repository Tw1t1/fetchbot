import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        
        self.frame_id = 0
        self.image_width = 640
        self.image_height = 480
        self.fps = 30
        
        self.image_pub = self.create_publisher(Image, '/camera_sensor/image_raw', 5)
        
        self.bridge = CvBridge()
        
        
        try:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        except Exception as e:
            self.get_logger().info(f'Failed to open camera: {str(e)}')
            return
        
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)
        
        self.get_logger().info('Camera node has been initialized')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Failed to capture frame')
            return
        
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.frame_id += 1

            img_msg.header.frame_id = str(self.frame_id)
            self.image_pub.publish(img_msg)
            
            # NOTE this is just for testing
            # self.get_logger().info(f'Published image id: {img_msg.header.frame_id}')

        except Exception as e:
            self.get_logger().info(f'Failed to convert/publish image: {str(e)}')
        
        
    def cleanup(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        self.get_logger().info('Node release capture and shutting down...')


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        camera_node.cleanup()
        camera_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()