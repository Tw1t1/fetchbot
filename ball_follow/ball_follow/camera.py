import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('frame_id', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30)
        
        self.camera_device = self.get_parameter('camera_device').value
        self.frame_id = self.get_parameter('frame_id').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        
        self.image_pub = self.create_publisher(Image, '/camera_sensor/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_sensor/camera_info', 10)
        
        self.bridge = CvBridge()
        
        self.camera_info = self.load_camera_info()
        
        try:
            self.cap = cv2.VideoCapture(self.camera_device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        except Exception as e:
            self.get_logger().error(f'Failed to open camera: {str(e)}')
            return
        
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)
        
        self.get_logger().info('Camera node has been initialized')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
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
            self.get_logger().error(f'Failed to convert/publish image: {str(e)}')
        
        self.camera_info.header = img_msg.header
        self.camera_info_pub.publish(self.camera_info)

    def load_camera_info(self):
        camera_info = CameraInfo()
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        camera_info.k = [1.0, 0.0, self.image_width/2, 0.0, 1.0, self.image_height/2, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [1.0, 0.0, self.image_width/2, 0.0, 0.0, 1.0, self.image_height/2, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_info.distortion_model = 'plumb_bob'
        return camera_info

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