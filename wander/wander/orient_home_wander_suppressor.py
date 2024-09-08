import rclpy
from fetchbot_interfaces.msg import Heading
from suppressor_inhibitor.suppressor import SuppressorNode

def main(args=None):
    rclpy.init(args=args)
    orient_home_wander_suppressor_node = SuppressorNode(
        name ='avoid_runaway_suppressor',
        sub1_msg_type = Heading,
        sub1_topic_name = 'orient_home/heading',
        sub2_msg_type = Heading,
        sub2_topic_name = 'follow_ball_wander_suppressor',
        pub_msg_type = Heading,
        pub_topic_name = 'orient_home_wander_suppressor',
        delay = 1.0
    )
    try:
        rclpy.spin(orient_home_wander_suppressor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            orient_home_wander_suppressor_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
