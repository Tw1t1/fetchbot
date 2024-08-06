
'''
Locomotion module is for diffrencial drive and converting Twist messages to Hardware commands based on
follow project : https://github.com/Reinbert/ros_diffdrive_robot/blob/master/ros_diffdrive_robot.ino
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hardware_librery.L298N import L298N  # Assuming the L298N class is in a file named l298n.py


class LocomotionController(Node):

    def __init__(self):
        super().__init__('locomotion_controller')

        # Robot parameters
        self.wheel_radius = 0.0525  # meters
        self.wheel_separation = 0.22  # meters

        # L298N setup
        try:
            self.left_motor = L298N(en=17, in1=22, in2=27)  # Adjust pins as needed
            self.right_motor = L298N(en=25, in1=24, in2=23)  # Adjust pins as needed
        except Exception as e:
            self.get_logger().error(f'Failed to initialize motors: {str(e)}')
            raise

        # ROS 2 setup
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback_b,
            10)
        
        self.get_logger().info('Locomotion Controller has been started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Wheel separation: {self.wheel_separation}m')

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity):
        """
        Calculate the velocities of the left and right wheels based on
        the desired linear and angular velocities of the robot.
        """
        v_linear = linear_velocity / self.wheel_radius
        v_angular = self.wheel_separation * angular_velocity / (2 * self.wheel_radius)

        left_wheel_velocity = v_linear - v_angular
        right_wheel_velocity = v_linear + v_angular

        return left_wheel_velocity, right_wheel_velocity
    
    def twist_callback_b(self, msg):
        """
        Callback function for handling Twist messages.
        Converts linear and angular velocities to motor commands.
        """
        try:
            # Extract linear and angular velocities
            v = msg.linear.x
            w = msg.angular.z

            self.get_logger().info(f'Received Twist: linear={v}, angular={w}')

            # Calculate wheel velocities
            left_wheel_velocity = (v - (w * self.wheel_separation / 2)) / self.wheel_radius
            right_wheel_velocity = (v + (w * self.wheel_separation / 2)) / self.wheel_radius
            
            # left_wheel_velocity, right_wheel_velocity = self.calculate_wheel_velocities(v, w)

            self.get_logger().info(f'Calculated wheel velocities: left={left_wheel_velocity}, right={right_wheel_velocity}')

            # Convert to duty cycle (0 to 100) and direction
            max_wheel_velocity = max(abs(left_wheel_velocity), abs(right_wheel_velocity))
            if max_wheel_velocity > 0:
                left_dc = abs(left_wheel_velocity / max_wheel_velocity) * 100
                right_dc = abs(right_wheel_velocity / max_wheel_velocity) * 100
                left_direction = 1 if left_wheel_velocity >= 0 else -1
                right_direction = 1 if right_wheel_velocity >= 0 else -1
            else:
                left_dc = right_dc = 0
                left_direction = right_direction = 0

            self.get_logger().info(f'Converted to duty cycles: left={left_dc}, right={right_dc}')
            self.get_logger().info(f'Directions: left={left_direction}, right={right_direction}')

            # Set motor directions and speeds
            self.set_motor_b(self.left_motor, left_dc, left_direction, 'left')
            self.set_motor_b(self.right_motor, right_dc, right_direction, 'right')

        except Exception as e:
            self.get_logger().error(f'Error in twist_callback: {str(e)}')
    def twist_callback(self, msg):
        #this function is same the above , but not consider the wheel radius and wheel seperaion
        
        # Cap values at [-1 .. 1]
        x = max(min(msg.linear.x, 1.0), -1.0)
        z = max(min(msg.angular.z, 1.0), -1.0)

        # Calculate the intensity of left and right wheels
        left = (x - z) / 2
        right = (x + z) / 2

        # Convert to duty cycle (0-100)
        left_dc = abs(int(left * 100))
        right_dc = abs(int(right * 100))

        # Set motor directions and speeds
        self.set_motor(self.left_motor, left, left_dc)
        self.set_motor(self.right_motor, right, right_dc)
        
    def set_motor(self, motor, value, duty_cycle):
        motor.set_duty_cycle(duty_cycle)
        if value > 0:
            motor.forward()
        elif value < 0:
            motor.backward()
        else:
            motor.stop()


    def set_motor_b(self, motor, duty_cycle, direction, motor_name):
        """
        Set the speed and direction of a motor.
        
        ################# WHEEL RADIUS, WHEEL SEPERARTION. ################# 
        ################# FIX NeGATIVE DUTY CYCLE. ######################### 
        
        :param motor: L298N motor object
        :param duty_cycle: Duty cycle (0 to 100)
        :param direction: Direction of rotation (1 for forward, -1 for backward, 0 for stop)
        :param motor_name: Name of the motor (for logging)
        """
        try:
            min_duty_cycle = 40 # NEW
            duty_cycle = max(min_duty_cycle, min(int(duty_cycle), 100))  # Ensure duty cycle is between 0 and 100
            motor.set_duty_cycle(duty_cycle)
            
            if direction > 0:
                motor.forward()
                self.get_logger().info(f'{motor_name.capitalize()} motor moving forward at {duty_cycle}% duty cycle')
            elif direction < 0:
                motor.backward()
                self.get_logger().info(f'{motor_name.capitalize()} motor moving backward at {duty_cycle}% duty cycle')
            else:
                motor.stop()
                self.get_logger().info(f'{motor_name.capitalize()} motor stopped')
        except Exception as e:
            self.get_logger().error(f'Error setting {motor_name} motor: {str(e)}')

    def stop(self):
        """Stop both motors."""
        try:
            self.left_motor.stop()
            self.right_motor.stop()
            self.get_logger().info('Both motors stopped')
        except Exception as e:
            self.get_logger().error(f'Error stopping motors: {str(e)}')


    def cleanup(self):
        """Cleanup function to stop motors and release GPIO pins."""
        try:
            self.stop()
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