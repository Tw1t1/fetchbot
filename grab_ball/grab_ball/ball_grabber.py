import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point

class GrabBall(Node):
    def __init__(self):
        super().__init__('grab_ball')
        self.position_sub = self.create_subscription(Float64, 'position', self.position_callback, 10)
        self.ball_info_sub = self.create_subscription(Point, '/detected_ball', self.ball_info_callback, 10)

        self.grab_ball_status_pub = self.create_publisher(String, 'grab_ball/status', 10)
        self.claw_cmd_pub = self.create_publisher(String, 'claw_cmd', 10)
        
        # Timer to publish grab ball staus
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)

        
        self.ball_info = None
        self.claw_state = 'open'
        self.grab_process_state = 'waiting'

        # Variables for position change detection
        self.current_position = None
        self.previous_position = None
        self.unchanged_position_count = 0
        self.position_unchanged = False
        self.position_change_threshold = 0.001  # Adjust this value based on your system's precision
        self.position_range_min = 2.0  # Minimum position to start tracking
        self.position_range_max = 3.0  # Maximum position to stop tracking
        
        

    def position_callback(self, msg):
        
        self.current_position = msg.data

        if self.position_range_min <= self.current_position <= self.position_range_max:
            if self.previous_position is not None:
                if abs(self.current_position - self.previous_position) < self.position_change_threshold:
                    self.unchanged_position_count += 1
                    if self.unchanged_position_count >= 4:
                        self.position_unchanged = True
                else:
                    self.unchanged_position_count = 0
                    self.position_unchanged = False
            self.previous_position = self.current_position
        else:
            # Reset tracking the position when outside the range
            self.unchanged_position_count = 0
            self.position_unchanged = False
            self.previous_position = None


    def ball_info_callback(self, msg):
        self.ball_info = msg

        ball_in_left_bottom = self.ball_info.x < 0 and self.ball_info.y > 0
        ball_in_right_bottom = self.ball_info.x > 0 and self.ball_info.y > 0
        
        if (ball_in_left_bottom or ball_in_right_bottom) and self.ball_info.z > 0.65:
            if not self.position_unchanged:
                self.start_closing_claw()
                self.claw_state = 'closing'
                self.grab_process_state = 'closing'
            else:
                self.stop_claw()
                self.claw_state = 'stopped'
                self.grab_process_state = 'grabbed'
        else:
            self.open_claw()
            self.claw_state = 'open'
            self.grab_process_state = 'waiting'


    def start_closing_claw(self):
        claw_cmd = String()
        claw_cmd.data = 'close'
        self.claw_cmd_pub.publish(claw_cmd)


    def stop_claw(self):
        claw_cmd = String()
        claw_cmd.data = 'stop'
        self.claw_cmd_pub.publish(claw_cmd)


    def open_claw(self):
        claw_cmd = String()
        claw_cmd.data = 'open'
        self.claw_cmd_pub.publish(claw_cmd)


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
