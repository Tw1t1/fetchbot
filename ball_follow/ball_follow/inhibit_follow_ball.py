import rclpy
from suppressor_inhibitor.inhibitor import InhibitorNode
from fetchbot_interfaces.msg import Heading
from std_msgs.msg import String


def check_not_waiting(msg):
    return msg.data != "WAITING"

def main(args=None):
    rclpy.init(args=args)
    follow_ball_inhibitor = InhibitorNode(
        name ='grab_ball_follow_ball_inhibitor',

        sub1_msg_type = String,
        sub1_topic_name = '/grab_ball/status',

        sub2_msg_type = Heading,
        sub2_topic_name = '/follow_ball',
        
        pub_msg_type = Heading,
        pub_topic_name = 'grab_ball_follow_ball_inhibitor',

        delay = 1.0,
        function = check_not_waiting
    )
    try:
        rclpy.spin(follow_ball_inhibitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        follow_ball_inhibitor.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        follow_ball_inhibitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

