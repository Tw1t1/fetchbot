# Based on Josh Newans code (modifications by Yosef Seada)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ball_follow.my_proc_image import BallDetector




class DetectBall(Node):

    def __init__(self):
        super().__init__('detect_ball')

        self.image_sub = self.create_subscription(
            Image, 
            "/camera_sensor/image_raw",
            self.callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
            )
        
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.ball_pub  = self.create_publisher(Point,"/detected_ball",1)

        self.declare_parameter('tuning_mode', False)

        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",0)
        self.declare_parameter("y_max",100)
        self.declare_parameter("h_min",0)
        self.declare_parameter("h_max",180)
        self.declare_parameter("s_min",0)
        self.declare_parameter("s_max",255)
        self.declare_parameter("v_min",0)
        self.declare_parameter("v_max",255)
        self.declare_parameter("sz_min",0)
        self.declare_parameter("sz_max",100)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value,
        }

        self.bridge = CvBridge()
        self.detector = BallDetector(self.tuning_params)

        if(self.tuning_mode):
            self.detector.create_tuning_window(self.tuning_params)
        
        self.get_logger().info('Looking for the ball...')


    def callback(self, data):
        try:
            # only for debugging and testing
            self.get_logger().info(f"Recived image")

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if (self.tuning_mode):
                self.tuning_params = self.detector.get_tuning_params()

            keypoints_norm, out_image, tuning_image = self.detector.find_circles(cv_image, self.tuning_mode)
            if self.tuning_mode:
                img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
                img_to_pub.header = data.header
                self.image_out_pub.publish(img_to_pub)

                img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
                img_to_pub.header = data.header
                self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            # Keep the biggest point, They are already converted to normalised coordinates
            for kp in keypoints_norm:                
                if (kp.size > point_out.z):                    
                    point_out.x = kp.pt[0]
                    point_out.y = kp.pt[1]
                    point_out.z = kp.size

            if (point_out.z > 0):
                self.ball_pub.publish(point_out)
                # only for debugging and testing
                self.get_logger().info(f"Ball detected: ({point_out.x}, {point_out.y}),  {point_out.z}")
        except Exception as e:
            self.get_logger().error(e)

    def cleanup(self):
        self.get_logger().info('Node shutting down...')


def main(args=None):
    rclpy.init(args=args)
    detect_ball = DetectBall()
    try:
        while rclpy.ok():
            rclpy.spin_once(detect_ball)
            BallDetector.wait_on_gui()
    except KeyboardInterrupt:
        pass
    finally:
        detect_ball.cleanup()
        detect_ball.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

