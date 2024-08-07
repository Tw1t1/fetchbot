
'''
Locomotion module is for diffrencial drive and converting Twist messages to Hardware commands based on
follow project : https://github.com/Reinbert/ros_diffdrive_robot/blob/master/ros_diffdrive_robot.ino
'''



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hardware_librery.L298N import L298N
import math

class LocomotionController(Node):

    def __init__(self):
        super().__init__('locomotion_controller')

        # Robot parameters
        self.wheel_radius = 0.0525  # meters
        self.wheel_separation = 0.22  # meters
        self.min_duty_cycle = 20  # Minimum duty cycle to ensure motor response

        # Maximum linear and angular velocities
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 2.0  # rad/s

        # Kinematic model correction factors
        self.K1 = 1.0  # General correction factor, can be calibrated
        self.K2 = 1.0  # Rotation correction factor, can be calibrated

        # Smoothing parameters
        self.max_acceleration = 0.5  # m/s^2, can be adjusted
        self.current_left_velocity = 0
        self.current_right_velocity = 0

        # L298N setup
        try:
            self.left_motor = L298N(en=17, in1=22, in2=27)
            self.right_motor = L298N(en=25, in1=24, in2=23)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize motors: {str(e)}')
            raise

        # ROS 2 setup
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        
        self.get_logger().info('Enhanced Locomotion Controller has been started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Wheel separation: {self.wheel_separation}m')

    def calculate_wheel_velocities(self, v, w):
        """
        Calculate wheel velocities using an enhanced kinematic model.
        
        This function applies correction factors K1 and K2 to improve the accuracy
        of the wheel velocity calculations, especially for turns.
        
        :param v: Linear velocity of the robot (m/s)
        :param w: Angular velocity of the robot (rad/s)
        :return: Tuple of (left_wheel_velocity, right_wheel_velocity)
        """
        left_velocity = self.K1 * (v - self.K2 * (w * self.wheel_separation / 2)) / self.wheel_radius
        right_velocity = self.K1 * (v + self.K2 * (w * self.wheel_separation / 2)) / self.wheel_radius
        return left_velocity, right_velocity

    def nonlinear_map(self, velocity):
        """
        Apply a nonlinear mapping to the velocity.
        
        This function can help compensate for non-linear motor responses.
        The current implementation uses a quadratic mapping, but this can be
        adjusted based on the specific behavior of your motors.
        
        :param velocity: Input velocity
        :return: Mapped velocity
        """
        max_velocity = self.max_linear_velocity / self.wheel_radius
        return math.copysign(velocity**2, velocity) * 100 / (max_velocity**2)

    def apply_velocity(self, target_velocity, current_velocity):
        """
        Apply velocity changes with acceleration limits and static friction compensation.
        
        This function implements smooth acceleration and deceleration, and
        compensates for static friction by ensuring a minimum duty cycle.
        
        :param target_velocity: Desired velocity
        :param current_velocity: Current velocity
        :return: Tuple of (duty_cycle, new_velocity)
        """
        new_velocity = self.ramp_to_target(current_velocity, target_velocity, self.max_acceleration)
        duty_cycle = self.nonlinear_map(new_velocity)
        if abs(new_velocity) > 0:
            duty_cycle = max(self.min_duty_cycle, abs(duty_cycle))
        return duty_cycle, new_velocity

    def ramp_to_target(self, current, target, max_change):
        """
        Gradually change the current value towards the target value.
        
        This function ensures smooth transitions in velocity, improving
        the overall motion of the robot.
        
        :param current: Current value
        :param target: Target value
        :param max_change: Maximum allowed change
        :return: New value after applying the ramp
        """
        diff = target - current
        if abs(diff) > max_change:
            return current + math.copysign(max_change, diff)
        return target

    def adjust_for_sharp_turn(self, left_velocity, right_velocity):
        """
        Adjust wheel velocities for sharp turns.
        
        This function implements a simple strategy for sharp turns by reversing
        one of the wheels if the difference in velocities is above a threshold.
        
        :param left_velocity: Velocity of the left wheel
        :param right_velocity: Velocity of the right wheel
        :return: Tuple of adjusted (left_velocity, right_velocity)
        """
        turn_threshold = 0.5  # This value can be calibrated
        if abs(left_velocity - right_velocity) > turn_threshold:
            if left_velocity > right_velocity:
                right_velocity = -right_velocity / 2
            else:
                left_velocity = -left_velocity / 2
        return left_velocity, right_velocity

    def twist_callback(self, msg):
        """
        Callback function for handling Twist messages.
        
        This function processes incoming velocity commands and converts them
        to appropriate motor controls, applying various enhancements for
        improved robot motion.
        """
        try:
            # Normalize velocities
            v = self.normalize_velocity(msg.linear.x, self.max_linear_velocity)
            w = self.normalize_velocity(msg.angular.z, self.max_angular_velocity)

            self.get_logger().info(f'Received Twist: linear={v}, angular={w}')

            # Calculate wheel velocities
            left_velocity, right_velocity = self.calculate_wheel_velocities(v, w)
            
            # Adjust for sharp turns if necessary
            left_velocity, right_velocity = self.adjust_for_sharp_turn(left_velocity, right_velocity)

            self.get_logger().info(f'Calculated wheel velocities: left={left_velocity}, right={right_velocity}')

            # Apply velocity changes with smoothing and convert to duty cycles
            left_dc, self.current_left_velocity = self.apply_velocity(left_velocity, self.current_left_velocity)
            right_dc, self.current_right_velocity = self.apply_velocity(right_velocity, self.current_right_velocity)

            # Determine directions
            left_direction = 1 if self.current_left_velocity >= 0 else -1
            right_direction = 1 if self.current_right_velocity >= 0 else -1

            self.get_logger().info(f'Converted to duty cycles: left={left_dc}, right={right_dc}')
            self.get_logger().info(f'Directions: left={left_direction}, right={right_direction}')

            # Set motor directions and speeds
            self.set_motor(self.left_motor, left_dc, left_direction, 'left')
            self.set_motor(self.right_motor, right_dc, right_direction, 'right')

        except Exception as e:
            self.get_logger().error(f'Error in twist_callback: {str(e)}')

    def normalize_velocity(self, velocity, max_velocity):
        """
        Normalize the input velocity to ensure it's within the allowed range.
        
        :param velocity: Input velocity
        :param max_velocity: Maximum allowed velocity
        :return: Normalized velocity
        """
        return max(min(velocity, max_velocity), -max_velocity)

    def set_motor(self, motor, duty_cycle, direction, motor_name):
        """
        Set the speed and direction of a motor.
        
        :param motor: L298N motor object
        :param duty_cycle: Duty cycle (0 to 100)
        :param direction: Direction of rotation (1 for forward, -1 for backward)
        :param motor_name: Name of the motor (for logging)
        """
        try:
            duty_cycle = max(0, min(int(abs(duty_cycle)), 100))  # Ensure duty cycle is between 0 and 100
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
        self.min_duty_cycle = 20  # Minimum duty cycle to ensure motor response


        # Maximum linear and angular velocities
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 2.0  # rad/s


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
            self.twist_callback,
            10)
        
        self.get_logger().info('Locomotion Controller has been started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Wheel separation: {self.wheel_separation}m')


    def calculate_wheel_velocities(self, v, w):
        """
        Calculate the velocities of the left and right wheels based on
        the desired linear and angular velocities of the robot.
        """
        left_wheel_velocity = (v - (w * self.wheel_separation / 2)) / self.wheel_radius
        right_wheel_velocity = (v + (w * self.wheel_separation / 2)) / self.wheel_radius
        return left_wheel_velocity, right_wheel_velocity
    
    def calculate_wheel_velocities(self, v, w):
        # Calculate wheel velocities based on the unicycle model
        """
        Calculate the velocities of the left and right wheels based on
        the desired linear and angular velocities of the robot.
        """
        left_wheel_velocity = (2*v - w*self.wheel_separation) / (2*self.wheel_radius)
        right_wheel_velocity = (2*v + w*self.wheel_separation) / (2*self.wheel_radius)
    
        
        return left_wheel_velocity, right_wheel_velocity
    
    def calculate_wheel_velocities_skid(self, v, w):
        """
        Calculate wheel velocities for a skid-steering robot.
        
        For a skid-steering robot:
        v_l = v - (w * L) / 2
        v_r = v + (w * L) / 2
        
        Where:
        v_l, v_r are the left and right side velocities
        L is the wheel separation (distance between left and right wheel pairs)
        """
        left_wheel_velocity = v - (w * self.wheel_separation / 2)
        right_wheel_velocity = v + (w * self.wheel_separation / 2)
        
        return left_wheel_velocity, right_wheel_velocity

    def normalize_velocity(self, velocity, max_velocity):
        return max(min(velocity, max_velocity), -max_velocity)
    
    
    def velocity_to_duty_cycle(self, velocity, max_wheel_velocity):
        """
        Convert wheel velocity to duty cycle and direction.
        """
        # max_wheel_velocity = (self.max_linear_velocity + (self.max_angular_velocity * self.wheel_separation / 2)) / self.wheel_radius
        if max_wheel_velocity != 0:
            duty_cycle = min(abs(velocity / max_wheel_velocity) * 100, 100)
            direction = 1 if velocity >= 0 else -1
        else:
            duty_cycle = direction = 0
        return duty_cycle, direction
    
    def twist_callback(self, msg):
        """
        Callback function for handling Twist messages.
        Converts linear and angular velocities to motor commands.
        """
        try:
            # Extract linear and angular velocities
            # Normalize velocities
            v = self.normalize_velocity(msg.linear.x, self.max_linear_velocity)
            w = self.normalize_velocity(msg.angular.z, self.max_angular_velocity)

            self.get_logger().info(f'Received Twist: linear={v}, angular={w}')

            # Calculate wheel velocities            
            left_wheel_velocity, right_wheel_velocity = self.calculate_wheel_velocities_skid(v, w)

            self.get_logger().info(f'Calculated wheel velocities: left={left_wheel_velocity}, right={right_wheel_velocity}')

            # Convert to duty cycle (0 to 100) and direction
            max_wheel_velocity = max(abs(left_wheel_velocity), abs(right_wheel_velocity))
            left_dc, left_direction = self.velocity_to_duty_cycle(left_wheel_velocity, max_wheel_velocity)
            right_dc, right_direction = self.velocity_to_duty_cycle(right_wheel_velocity, max_wheel_velocity)

            # Convert to duty cycle (0 to 100) and direction
            # max_wheel_velocity = max(abs(left_wheel_velocity), abs(right_wheel_velocity))
            # if max_wheel_velocity > 0:
                
            #     left_dc = abs(left_wheel_velocity / max_wheel_velocity) * 100
            #     right_dc = abs(right_wheel_velocity / max_wheel_velocity) * 100
                
            #     left_direction = 1 if left_wheel_velocity >= 0 else -1
            #     right_direction = 1 if right_wheel_velocity >= 0 else -1
            # else:
            #     left_dc = right_dc = 0
            #     left_direction = right_direction = 0

            self.get_logger().info(f'Converted to duty cycles: left={left_dc}, right={right_dc}')
            self.get_logger().info(f'Directions: left={left_direction}, right={right_direction}')

            # Set motor directions and speeds
            self.set_motor_b(self.left_motor, left_dc, left_direction, 'left')
            self.set_motor_b(self.right_motor, right_dc, right_direction, 'right')

        except Exception as e:
            self.get_logger().error(f'Error in twist_callback: {str(e)}')


    def velocity_to_duty_cycle(self, velocity):
        """
        Convert wheel velocity to duty cycle and direction.
        """
        max_wheel_velocity = (self.max_linear_velocity + (self.max_angular_velocity * self.wheel_separation / 2)) / self.wheel_radius
        duty_cycle = min(abs(velocity / max_wheel_velocity) * 100, 100)
        direction = 1 if velocity >= 0 else -1
        return duty_cycle, direction
    
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
            duty_cycle = max(self.min_duty_cycle, min(int(duty_cycle), 100))  # Ensure duty cycle is between 0 and 100
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
'''

















    # def twist_callback(self, msg):
    #     #this function is same the above , but not consider the wheel radius and wheel seperaion
        
    #     # Cap values at [-1 .. 1]
    #     x = max(min(msg.linear.x, 1.0), -1.0)
    #     z = max(min(msg.angular.z, 1.0), -1.0)

    #     # Calculate the intensity of left and right wheels
    #     left = (x - z) / 2
    #     right = (x + z) / 2

    #     # Convert to duty cycle (0-100)
    #     left_dc = abs(int(left * 100))
    #     right_dc = abs(int(right * 100))

    #     # Set motor directions and speeds
    #     self.set_motor(self.left_motor, left, left_dc)
    #     self.set_motor(self.right_motor, right, right_dc)
        
    # def set_motor(self, motor, value, duty_cycle):
    #     motor.set_duty_cycle(duty_cycle)
    #     if value > 0:
    #         motor.forward()
    #     elif value < 0:
    #         motor.backward()
    #     else:
    #         motor.stop()