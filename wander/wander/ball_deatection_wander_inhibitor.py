import rclpy
from fetchbot_interfaces.msg import Heading, BallInfo
from suppressor_inhibitor.suppressor import SuppressorNode

def main(args=None):
    rclpy.init(args=args)
    ball_deatection_wander_inhibitor_node = SuppressorNode(
        name ='avoid_runaway_suppressor',
        sub1_msg_type = BallInfo,
        sub1_topic_name = 'orient_home_ball_detection_inhibitore',
        sub2_msg_type = Heading,
        sub2_topic_name = 'wander',
        pub_msg_type = Heading,
        pub_topic_name = 'ball_deatection_wander_inhibitor',
        delay = 0.5
    )
    try:
        rclpy.spin(ball_deatection_wander_inhibitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            ball_deatection_wander_inhibitor_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
