import rclpy
from fetchbot_interfaces.msg import Heading
from suppressor_inhibitor.suppressor import SuppressorNode

def main(args=None):
    rclpy.init(args=args)
    follow_ball_wander_suppressor_node = SuppressorNode(
        name ='avoid_runaway_suppressor',
        sub1_msg_type = Heading,
        sub1_topic_name = 'grab_ball_follow_ball_inhibitor',
        sub2_msg_type = Heading,
        sub2_topic_name = 'detect_ball_wander_inhibitor',
        pub_msg_type = Heading,
        pub_topic_name = 'follow_ball_wander_suppressor',
        delay = 0.5
    )
    try:
        rclpy.spin(follow_ball_wander_suppressor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            follow_ball_wander_suppressor_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
