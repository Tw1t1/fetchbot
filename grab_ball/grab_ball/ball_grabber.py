import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from enum import Enum


class GrabBallStatus(Enum):
    WAITING = 0
    CLOSING = 1
    OPENING = 2
    GRABBED = 3 


class BallSizeStatus(Enum):
    NORMAL = 0
    SUSPICIOUS = 1
    INVALID = 2


class GrabBall(Node):
    def __init__(self):
        super().__init__('grab_ball')
        self.position_sub = self.create_subscription(Float64, 'position', self.position_callback, 10)
        self.ball_info_sub = self.create_subscription(Point, '/detected_ball', self.ball_info_callback, 10)

        self.grab_ball_status_pub = self.create_publisher(String, 'grab_ball/status', 10)
        self.claw_cmd_pub = self.create_publisher(String, 'claw_cmd', 10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)

        self.ball_info = None
        self.claw_state = 'open'
        self.grab_process_state = GrabBallStatus.WAITING

        # Variables for position change detection
        self.current_position = None
        self.previous_position = None
        self.unchanged_position_count = 0
        self.position_unchanged = False
        self.position_change_threshold = 0.01
        self.position_range_min = 3.0
        self.position_range_max = 4.0
        
        # New variables for ball size analysis
        self.ball_size_normal_min = 0.6 # 0.5
        self.ball_size_normal_max = 0.9 
        self.ball_size_threshold_min = 0.4
        self.ball_size_threshold_max = 0.95

        self.claw_cmd = String()

        self.get_logger().info('Waiting for ball to grab ... ')

    def position_callback(self, msg):
        try:
            self.current_position = round(msg.data, 3)

            if self.grab_process_state == GrabBallStatus.WAITING and self.current_position > 0.5 \
                or self.current_position > 4.0:
                self.open_claw()
                self.claw_state = 'open'
                self.grab_process_state = GrabBallStatus.WAITING


            ball_in_grabe_position_range = self.position_range_min <= self.current_position <= self.position_range_max
            
            if ball_in_grabe_position_range and self.grab_process_state == GrabBallStatus.CLOSING:
                if self.previous_position is not None:
                    self.get_logger().info(f'Position - Previuse = {abs(self.current_position - self.previous_position)}')
                    if abs(self.current_position - self.previous_position) <= self.position_change_threshold:
                        self.unchanged_position_count += 1
                        if self.unchanged_position_count >= 5:
                            self.position_unchanged = True
                    else:
                        self.unchanged_position_count = 0
                        self.position_unchanged = False
                self.previous_position = self.current_position
            else:
                self.unchanged_position_count = 0
                self.position_unchanged = False
                self.previous_position = None
        except Exception as e:
            self.get_logger().error(f'Error in position_callback: {str(e)}')

    def ball_info_callback(self, msg):
        try:
            self.ball_info = msg
            
            ball_size_status = self.check_ball_size(self.ball_info.x, self.ball_info.y, self.ball_info.z)
            
            if ball_size_status == BallSizeStatus.NORMAL:
                self.get_logger().info(f'Ball to grab: pos ({round(self.ball_info.x, 5)}, {round(self.ball_info.y, 5)}) , size {round(self.ball_info.z, 5)}')

                grab_status = String()
                grab_status.data = self.grab_process_state.name
                self.grab_ball_status_pub.publish(grab_status)

                if not self.position_unchanged:
                    self.close_claw()
                    self.claw_state = 'closing'
                    self.grab_process_state = GrabBallStatus.CLOSING
                else:
                    self.stop_claw()
                    self.claw_state = 'stopped'
                    self.grab_process_state = GrabBallStatus.GRABBED
            else:
                self.open_claw()
                self.claw_state = 'open'
                self.grab_process_state = GrabBallStatus.WAITING

            self.get_logger().info(f'grab ball process state {self.grab_process_state}, claw state {self.claw_state}, ball size status {ball_size_status}')

        except Exception as e:
            self.get_logger().error(f'Error in ball_info_callback: {str(e)}')

    def check_ball_size(self, x, y, size):

        x_min, x_max = -0.21, 0.6
        y_min, y_max = 0.40, 0.62
        size_min, size_max = 0.69, 0.87
        # ball_in_left_bottom = self.ball_info.x <= x_min and self.ball_info.y <= y_max
        # ball_in_right_bottom = self.ball_info.x <= x_max and self.ball_info.y < 0
        
        if x_min <= x <= x_max and y_min <= y <= y_max and size_min <= size <= size_max:
            return BallSizeStatus.NORMAL
        elif (-0.21 <= x <= 0.6) and (0.0 <= y <= 0.93) and (0.39 <= size <= 0.97):
            self.get_logger().warn(f'Ball size {size} is outside normal range but within thresholds')
            return BallSizeStatus.SUSPICIOUS
        else:
            self.get_logger().error(f'Invalid ball size detected: {size}')
            return BallSizeStatus.INVALID
        
        # if self.ball_size_normal_min <= size <= self.ball_size_normal_max:
        #     return BallSizeStatus.NORMAL
        # elif self.ball_size_threshold_min <= size < self.ball_size_normal_min or self.ball_size_normal_max < size <= self.ball_size_threshold_max:
        #     self.get_logger().warn(f'Ball size {size} is outside normal range but within thresholds')
        #     return BallSizeStatus.SUSPICIOUS
        # else:
        #     self.get_logger().error(f'Invalid ball size detected: {size}')
        #     return BallSizeStatus.INVALID

    def close_claw(self):
        self.claw_cmd.data = 'close'
        self.claw_cmd_pub.publish(self.claw_cmd)

    def stop_claw(self):
        self.claw_cmd.data = 'stop'
        self.claw_cmd_pub.publish(self.claw_cmd)

    def open_claw(self):
        self.claw_cmd.data = 'open'
        self.claw_cmd_pub.publish(self.claw_cmd)

    def publish_status(self):
        status = String()
        status.data = f"Claw: {self.claw_state}, Process: {self.grab_process_state}"
        self.grab_ball_status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    grab_ball_node = GrabBall()
    rclpy.spin(grab_ball_node)
    grab_ball_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()