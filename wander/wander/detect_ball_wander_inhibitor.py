import rclpy
from fetchbot_interfaces.msg import Heading, BallInfo
from suppressor_inhibitor.inhibitor import InhibitorNode

def main(args=None):
    rclpy.init(args=args)
    detect_ball_wander_inhibitor_node = InhibitorNode(
        name ='avoid_runaway_suppressor',
        sub1_msg_type = BallInfo,
        sub1_topic_name = 'orient_home_detect_ball_inhibitor',
        sub2_msg_type = Heading,
        sub2_topic_name = 'wander',
        pub_msg_type = Heading,
        pub_topic_name = 'detect_ball_wander_inhibitor',
        delay = 0.5
    )
    try:
        rclpy.spin(detect_ball_wander_inhibitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            detect_ball_wander_inhibitor_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
