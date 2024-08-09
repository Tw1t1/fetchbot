import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from enum import Enum
import time

class GrabStatus(Enum):
    WAITING = 0
    CLOSING = 1
    GRABBED = 2
    OPENING = 3 # not in use now


class GrabBall(Node):
    def __init__(self):
        super().__init__('grab_ball')
        self.position_sub = self.create_subscription(Float64, 'position', self.position_callback, 10)
        self.ball_info_sub = self.create_subscription(Point, '/detected_ball', self.ball_info_callback, 10)

        self.grab_ball_status_pub = self.create_publisher(String, 'grab_ball/status', 10)
        self.claw_cmd_pub = self.create_publisher(String, 'claw_cmd', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.rcv_timeout_secs = 1.5
        self.lastrcvtime = time.time() - 10000
        
        self.grab_process_state = GrabStatus.WAITING

        self.current_position = None
        self.previous_position = None
        self.unchanged_position_count = 0
        self.position_change_threshold = 0.1
        self.position_range_min = 3.0
        self.position_range_max = 4.0
        self.stable_position_count_threshold = 3

        # Varibels for ball info to grab
        self.ball_size_min = 0.65
        self.ball_size_max = 0.9
        self.x_min, self.x_max = -0.25, 0.6
        self.y_min, self.y_max = 0.40, 0.65

        self.ball_info_threshold = 0.05 # not in use now

        self.claw_cmd = String()
        self.status = String()

        self.get_logger().info('Waiting for ball to grab ... ')


    def position_callback(self, msg):
        try:
            self.current_position = round(msg.data, 3)

            if self.grab_process_state == GrabStatus.WAITING and self.current_position > 0.5:
                self.open_claw()
                self.get_logger().info(f'Claw opened. Current position: {self.current_position}')

            if self.is_ball_grabbed():
                self.stop_claw()
                self.grab_process_state = GrabStatus.GRABBED
                self.get_logger().info('Ball grabbed. Claw stopped.')
        except Exception as e:
            self.get_logger().error(f'Error in position_callback: {str(e)}')


    def ball_info_callback(self, msg):

        if msg is None:
            self.get_logger().warn('No ball information received yet.')
            return
        
        self.lastrcvtime = time.time()
        try:
                
            if self.should_grab_ball(msg.x, msg.y, msg.z):
                self.close_claw()
                self.grab_process_state = GrabStatus.CLOSING
                self.get_logger().info('Ball detected. Closing claw.')
            elif self.grab_process_state == GrabStatus.CLOSING and self.current_position > self.position_range_max:
                self.open_claw()
                self.grab_process_state = GrabStatus.WAITING
                self.get_logger().info('Missing grab Ball. Opening claw.')

        except Exception as e:
            self.get_logger().error(f'Error in ball_info_callback: {str(e)}')

    def should_grab_ball(self, x, y, size):
        
        if GrabStatus.GRABBED == self.grab_process_state:
            return False
        
        position_in_range = self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max
        size_in_range = self.ball_size_min <= size <= self.ball_size_max

        if position_in_range and size_in_range:
            self.get_logger().debug(f'Ball detected at position ({x}, {y}) with size {size}. Should grab.')
            return True
        else:
            self.get_logger().debug(f'Ball detected at position ({x}, {y}) with size {size}. Should not grab.')
            return False


    def is_ball_grabbed(self):
        if self.grab_process_state != GrabStatus.CLOSING:
            self.get_logger().debug(f"Not in CLOSING state. Current state: {self.grab_process_state}")
            return False

        is_position_in_grab_range = self.position_range_min <= self.current_position <= self.position_range_max

        if not is_position_in_grab_range:
            self.get_logger().debug(f"Current position {self.current_position} is outside the grab range [{self.position_range_min}, {self.position_range_max}]")
            self.reset_grab_detection()
            return False

        if self.previous_position is None:
            self.get_logger().debug("No previous position recorded. Starting grab detection.")
            self.previous_position = self.current_position
            return False

        position_change = round(abs(self.current_position - self.previous_position), 3)
        is_position_stable = position_change <= self.position_change_threshold

        if is_position_stable:
            self.unchanged_position_count += 1
            self.get_logger().debug(f"Position is stable. Unchanged count: {self.unchanged_position_count}, Position change: {position_change}")
            is_grab_detected = self.unchanged_position_count >= self.stable_position_count_threshold
            if is_grab_detected:
                self.get_logger().info("Ball grabbed detect!")
                return True
        else:
            self.get_logger().debug(f"Position change {position_change} exceeds the threshold {self.position_change_threshold}. Resetting grab detection.")
            self.reset_grab_detection()

        self.previous_position = self.current_position
        return False


    def reset_grab_detection(self):
        self.unchanged_position_count = 0
        self.previous_position = None
        self.get_logger().debug("Grab detection reset.")


    def close_claw(self):
        self.claw_cmd.data = 'close'
        self.claw_cmd_pub.publish(self.claw_cmd)
        self.get_logger().debug('Claw close command sent.')


    def stop_claw(self):
        self.claw_cmd.data = 'stop'
        self.claw_cmd_pub.publish(self.claw_cmd)
        self.get_logger().debug('Claw stop command sent.')


    def open_claw(self):
        self.claw_cmd.data = 'open'
        self.claw_cmd_pub.publish(self.claw_cmd)
        self.get_logger().debug('Claw open command sent.')


    def timer_callback(self):

        current_time = time.time()    
        if self.grab_process_state == GrabStatus.GRABBED and \
            (current_time - self.lastrcvtime) < self.rcv_timeout_secs:
            self.get_logger().info("Ball released!")
            self.open_claw()
            self.grab_process_state = GrabStatus.WAITING
        
        self.status.data = self.grab_process_state.name
        self.grab_ball_status_pub.publish(self.status)


def main(args=None):

    rclpy.init(args=args)
    try:
        grab_ball_node = GrabBall()
        rclpy.spin(grab_ball_node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        grab_ball_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()