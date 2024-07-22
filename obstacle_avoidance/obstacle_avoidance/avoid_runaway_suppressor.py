import rclpy
from geometry_msgs.msg import Twist
from fetchbot_interfaces.msg import Heading
from suppressor_inhibitor.suppressor import SuppressorNode

# Convert the Heading message to a Twist message
def convert_heading_to_twist(heading):
    twist = Twist()
    twist.linear.x = heading.distance
    twist.angular.z = heading.angle
    return twist

def main(args=None):
    rclpy.init(args=args)
    avoid_runaway_suppressor_node = SuppressorNode(
        name ='avoid_runaway_suppressor',
        sub1_msg_type = Heading,
        sub1_topic_name = 'avoid',
        sub2_msg_type = Heading,
        sub2_topic_name = 'runaway',
        pub_msg_type = Twist,
        pub_topic_name = 'diff_cont/cmd_vel_unstamped',
        delay = 0.5,
        publish_function=convert_heading_to_twist
    )
    try:
        rclpy.spin(avoid_runaway_suppressor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            avoid_runaway_suppressor_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
