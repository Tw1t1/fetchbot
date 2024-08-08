import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hardware_librery.L298N import L298N

class LocomotionController(Node):

    def __init__(self):
        super().__init__('locomotion_controller')

        # Robot parameters
        self.wheel_radius = 0.0525  # meters
        self.wheel_separation = 0.22  # meters
        self.min_duty_cycle = 30  # Minimum duty cycle to ensure motor response

        # L298N setup
        try:
            self.left_motor = L298N(en=17, in1=22, in2=27)
            self.right_motor = L298N(en=25, in1=24, in2=23)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize motors: {str(e)}')
            raise

        # Variables to store current velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Flag to alternate between linear and angular velocity
        self.use_linear_velocity = True

        # ROS 2 setup
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        
        self.timer = self.create_timer(0.1, self.motor_command_timer)

        self.get_logger().info('Locomotion Controller has been started')

    def twist_callback(self, msg):
        """Update current linear and angular velocities."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f'Received Twist: linear={self.linear_velocity}, angular={self.angular_velocity}')

    def motor_command_timer(self):
        """Send commands to motors every 0.1 seconds."""   
        try:
            
            if self.linear_velocity != 0 and self.angular_velocity != 0:
                if self.use_linear_velocity:
                    self.set_velocity(self.linear_velocity, 0)
                else:
                    self.set_velocity(0, self.angular_velocity)
                
                # Toggle between linear and angular velocity
                self.use_linear_velocity = not self.use_linear_velocity
            else:
                self.set_velocity(self.linear_velocity, self.angular_velocity)
            
        except Exception as e:
            self.get_logger().error(f'Error in motor_command_timer: {str(e)}')

    def set_velocity(self, linear, angular):
        """Set motor commands based on linear and angular velocity."""
        left_velocity = linear - (angular * self.wheel_separation / 2)
        right_velocity = linear + (angular * self.wheel_separation / 2)
        
        max_velocity = max(abs(left_velocity), abs(right_velocity), 1e-6)  # Avoid division by zero
        
        left_duty_cycle = self.velocity_to_duty_cycle(left_velocity, max_velocity)
        right_duty_cycle = self.velocity_to_duty_cycle(right_velocity, max_velocity)
        
        self.set_motor(self.left_motor, left_duty_cycle, left_velocity, 'left')
        self.set_motor(self.right_motor, right_duty_cycle, right_velocity, 'right')

    def velocity_to_duty_cycle(self, velocity, max_velocity):
        """Convert velocity to duty cycle."""
        duty_cycle = min(abs(velocity / max_velocity) * 100, 100)
        return max(self.min_duty_cycle, int(duty_cycle)) if abs(velocity) > 1e-6 else 0

    def set_motor(self, motor, duty_cycle, velocity, motor_name):
        """Set the speed and direction of a motor."""
        try:
            if abs(velocity) <= 1e-6:  # Stop condition
                motor.stop()
                self.get_logger().info(f'{motor_name.capitalize()} motor stopped')
            else:
                motor.set_duty_cycle(duty_cycle)
                if velocity > 0: # Direction check by velocity sign
                    motor.forward()
                    direction = "forward"
                else:
                    motor.backward()
                    direction = "backward"
                self.get_logger().info(f'{motor_name.capitalize()} motor moving {direction} at {duty_cycle}% duty cycle')
        except Exception as e:
            self.get_logger().error(f'Error setting {motor_name} motor: {str(e)}')

    def cleanup(self):
        """Cleanup function to stop motors and release GPIO pins."""
        try:
            self.set_velocity(0, 0)  # Stop both motors
            self.left_motor.cleanup()
            self.right_motor.cleanup()
            self.get_logger().info('Cleanup completed')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    controller = LocomotionController()
    try:
        rclpy.spin(controller)
    except Exception as e:
        print(f'Error in locomotion controller main: {str(e)}')
    finally:
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()