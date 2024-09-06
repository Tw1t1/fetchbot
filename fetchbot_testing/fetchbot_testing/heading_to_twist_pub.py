import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading
from geometry_msgs.msg import Twist


    

class TwistPubNode(Node):
    def __init__(self):
        super().__init__('twist_publisher')

        self.subscriber_ = self.create_subscription(
            Heading, '/follow_ball', self.heading_callback, 10)
        
        self.publisher_ = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', 10)
            

    def heading_callback(self, heading):
        msg = Twist()
        msg.linear.x = heading.distance
        msg.angular.z = heading.angle

        self.publisher_.publish(msg)

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