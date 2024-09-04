import rclpy
from fetchbot_interfaces.msg import BallInfo
from suppressor_inhibitor.inhibitor import InhibitorNode
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    detect_ball_inhibitor = InhibitorNode(
        name ='orient_home_detect_ball_inhibitor',

        sub1_msg_type = String,
        sub1_topic_name = '/orient_home/returning_home',

        sub2_msg_type = BallInfo,
        sub2_topic_name = 'detected_ball',
        
        pub_msg_type = BallInfo,
        pub_topic_name = 'orient_home_detect_ball_inhibitor',
        delay = 1.0
    )
    try:
        rclpy.spin(detect_ball_inhibitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        detect_ball_inhibitor.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        detect_ball_inhibitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
