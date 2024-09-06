import rclpy
from rclpy.node import Node
from std_msgs.msg import String
    

class TwistPubNode(Node):
    def __init__(self):
        super().__init__('orient_home')

        self.subscriber_ = self.create_subscription(
            String, '/grab_ball/status', self.callback, 2)
        
        self.publisher_ = self.create_publisher(
            String, '/orient_home_detect_ball_inhibitor', 2)

    def callback(self, msg):
        home_msg = String()
        home_msg.data = "HOME"

        if msg.data == "GRABBED":
            self.publisher_.publish(home_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistPubNode()
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