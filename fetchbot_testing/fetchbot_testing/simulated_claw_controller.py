import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

class SimulatedClawController(Node):

    def __init__(self):
        super().__init__('simulated_claw_controller')
        
        self.claw_cmd_sub = self.create_subscription(String, 'claw_cmd', self.claw_cmd_callback, 10)
        self.position_pub = self.create_publisher(Float64, 'position', 10)

        self.create_timer(0.1, self.timer_position_pub)
        self.initialize_parameters()
        
        self.current_position = 0.0  # Starting with open position
        self.claw_moving = False
        self.claw_direction = 0  # 0: stopped, 1: opening, -1: closing

        self.get_logger().info('Simulated claw_controller has been started!')

    def initialize_parameters(self):
        self.declare_parameter('claw_sensor_threshold', 10)
        self.claw_sensor_threshold = self.get_parameter('claw_sensor_threshold').get_parameter_value().integer_value
        self.claw_speed = 8.0  # Units per second

    def timer_position_pub(self):
        if self.claw_moving:
            # Update position based on direction and speed
            new_position = self.current_position + (self.claw_direction * self.claw_speed * 0.1)
            self.current_position = max(0.0, min(100.0, new_position))

            # Check if reached limits
            if self.current_position <= 0.0 or self.current_position >= 75.0: # for testing ball grabbed >= 75.0
                self.claw_moving = False
                self.claw_direction = 0
                self.get_logger().info(f"Claw reached {'open' if self.current_position <= 0 else 'closed'} position")

        msg = Float64()
        msg.data = self.current_position
        self.position_pub.publish(msg)

    def claw_cmd_callback(self, msg):
        command = msg.data
        if (self.current_position > self.claw_sensor_threshold) or \
           (self.current_position < 100.0 - self.claw_sensor_threshold):
            
            if command == 'open':
                self.claw_moving = True
                self.claw_direction = -1
                self.get_logger().info('Command received: Opening claw')
            elif command == 'close':
                self.claw_moving = True
                self.claw_direction = 1
                self.get_logger().info('Command received: Closing claw')
            elif command == 'stop':
                self.claw_moving = False
                self.claw_direction = 0
                self.get_logger().info('Command received: Stopping claw')
            else:
                self.get_logger().warn(f'Received invalid claw command: {command}')
        else:
            self.claw_moving = False
            self.claw_direction = 0
            self.get_logger().info('Claw at limit, stopping movement')

def main(args=None):
    rclpy.init(args=args)
    simulated_claw_controller = SimulatedClawController()
    try:
        rclpy.spin(simulated_claw_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        simulated_claw_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
