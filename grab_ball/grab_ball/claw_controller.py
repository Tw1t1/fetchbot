import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

from hardware_librery.ADCReader import ADCReader
from hardware_librery.L298N import L298N

class ClawController(Node):

    def __init__(self):
        try:
            super().__init__('claw_controller')
            self.get_logger().info('Initializing ClawController...')

            self.init_subscriptions()
            self.init_publishers()
            self.init_parameters()
            self.init_hardware()
            self.init_timer()

            self.get_logger().info('ClawController initialization complete.')
        except Exception as e:
            self.get_logger().error(f'Error in ClawController initialization: {str(e)}')
            raise

    def init_subscriptions(self):
        self.get_logger().info('Setting up subscriptions...')
        self.claw_cmd_sub = self.create_subscription(String, 'claw_cmd', self.claw_cmd_callback, 10)
        self.get_logger().info('Subscriptions set up successfully.')

    def init_publishers(self):
        self.get_logger().info('Setting up publishers...')
        self.position_pub = self.create_publisher(Float64, 'position', 10)
        self.get_logger().info('Publishers set up successfully.')

    def init_parameters(self):
        self.get_logger().info('Declaring and getting parameters...')
        self.declare_parameter('in1_pin', 21)
        self.declare_parameter('in2_pin', 20)
        self.declare_parameter('min_claw_value', 0.0)
        self.declare_parameter('max_claw_value', 5.0)

        self.IN1 = self.get_parameter('in1_pin').get_parameter_value().integer_value
        self.IN2 = self.get_parameter('in2_pin').get_parameter_value().integer_value
        self.MIN_VALUE = self.get_parameter('min_claw_value').get_parameter_value().double_value
        self.MAX_VALUE = self.get_parameter('max_claw_value').get_parameter_value().double_value

        self.get_logger().info(f'Parameters set: IN1={self.IN1}, IN2={self.IN2}, MIN_VALUE={self.MIN_VALUE}, MAX_VALUE={self.MAX_VALUE}')

    def init_hardware(self):
        self.get_logger().info('Initializing hardware components...')
        try:
            self.motor = L298N(en=None, in1=self.IN1, in2=self.IN2)
            self.get_logger().info('L298N motor driver initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Error initializing L298N motor driver: {str(e)}')
            raise

        try:
            self.adc = ADCReader()
            self.get_logger().info('ADC reader initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Error initializing ADC reader: {str(e)}')
            raise

    def init_timer(self):
        self.get_logger().info('Setting up timer...')
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_position_pub)
        self.position_msg = Float64()
        self.get_logger().info('Timer set up successfully.')

    def timer_position_pub(self):
        try:
            self.position_msg.data = self.read_potentiometer()
            self.position_pub.publish(self.position_msg)
            self.get_logger().debug(f'Published claw position: {self.position_msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error in timer_position_pub: {str(e)}')

    def claw_cmd_callback(self, msg):
        try:
            if msg.data == 'open':
                self.move_claw(1)
            elif msg.data == 'close':
                self.move_claw(-1)
            else:
                self.get_logger().warn(f'Received invalid claw command: {msg.data}')
            
            self.get_logger().info(f'Processed claw command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error in claw_cmd_callback: {str(e)}')

    def move_claw(self, direction):
        try:
            current_position = self.read_potentiometer()
            self.get_logger().debug(f'Current position: {current_position}, Direction: {direction}')

            if (direction > 0 and current_position < self.MAX_VALUE) or \
               (direction < 0 and current_position > self.MIN_VALUE):
                if direction > 0:
                    self.motor.forward()
                    self.get_logger().debug('Moving claw forward')
                else:
                    self.motor.backward()
                    self.get_logger().debug('Moving claw backward')
                self.motor.set_duty_cycle(50)
            else:
                self.motor.stop()
                self.get_logger().debug('Claw movement stopped (at limit or no movement needed)')
        except Exception as e:
            self.get_logger().error(f'Error in move_claw: {str(e)}')

    def read_potentiometer(self):
        try:
            with self.adc:
                voltage = self.adc.get_adc(0)
            self.get_logger().debug(f'Read potentiometer value: {voltage}')
            return voltage
        except Exception as e:
            self.get_logger().error(f'Error in read_potentiometer: {str(e)}')
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    try:
        claw_controller_node = ClawController()
        rclpy.spin(claw_controller_node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        if 'claw_controller_node' in locals():
            claw_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float64 # TODO modify custom_interfaces

# from hardware_librery.ADCReader import ADCReader
# from hardware_librery.L298N import L298N

# class ClawController(Node):

#     def __init__(self):
        
#         super().__init__('claw_controller')
#         self.claw_cmd_sub = self.create_subscription(String, 'claw_cmd', self.claw_cmd_callback, 10)
#         self.position_pub = self.create_publisher(Float64, 'position', 10)


#         self.declare_parameter('in1_pin', 21)
#         self.declare_parameter('in2_pin', 20)
    
#         self.declare_parameter('min_claw_value', 0.0)
#         self.declare_parameter('max_claw_value', 5.0)
        
#         # L298N pins numbers
#         self.IN1 = self.get_parameter('in1_pin').get_parameter_value().integer_value
#         self.IN2 = self.get_parameter('in2_pin').get_parameter_value().integer_value

#         # Potentiometer limits
#         self.MIN_VALUE = self.get_parameter('min_claw_value').get_parameter_value().double_value
#         self.MAX_VALUE = self.get_parameter('max_claw_value').get_parameter_value().double_value

#         # Timer to publish claw position
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_position_pub)
#         self.position_msg = Float64()

#         # L298N motor driver setup
#         self.motor = L298N(en=None, in1=self.IN1, in2=self.IN2) 

#         # ADC setup, using for read Potentiometer values 
#         self.adc = ADCReader()

#         self.get_logger().info(f'claw_controller has been start with :{self.adc}, {self.motor}')

#     def timer_position_pub(self):
#         try: 
#             self.position_msg.data = self.read_potentiometer()
#             self.position_pub.publish(self.position_msg)
#             self.get_logger().info(f'claw position: {self.position_msg.data}')
#         except Exception as e:
#             self.get_logger().error(f'Error in timer_position_pub: {str(e)}')


#     def claw_cmd_callback(self, msg):
#         try:
#             if msg.data == 'open':
#                 self.move_claw(1)  # Move in positive direction
#             elif msg.data == 'close':
#                 self.move_claw(-1)  # Move in negative direction
            
#             self.get_logger().info(f'claw command: {msg.data}')
#         except Exception as e:
#             self.get_logger().error(f'Error in claw_cmd_callback: {str(e)}')

#     def move_claw(self, direction):

#         try:
#             current_position = self.read_potentiometer()

#             if (direction > 0 and current_position < self.MAX_VALUE) or \
#             (direction < 0 and current_position > self.MIN_VALUE):
#                 # Set motor direction and speed
#                 if direction > 0:
#                     self.motor.forward()
#                 else:
#                     self.motor.backward()
#                 self.motor.set_duty_cycle(50)  # 50% duty cycle
#             else:
#                 # Stop motor if at limit or direction == 0
#                 self.motor.stop()
#         except Exception as e:
#             self.get_logger().error(f'Error in move_claw: {str(e)}')

#     def read_potentiometer(self):
#         # Read ADC value from channel 0
#         try:
#             with self.adc:  # Using the new context manager
#                 voltage = self.adc.get_adc(0)
#         except Exception as e:
#             self.get_logger().error(f'Error in read_potentiometer: {str(e)}')
#         return voltage

#     # def cleanup(self):
#     #     del self.motor  # This will clean up GPIO pins used by L298N

# def main(args=None):
#     rclpy.init(args=args)
#     claw_controller_node = ClawController()
#     rclpy.spin(claw_controller_node)
#     claw_controller_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
