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

            self.claw_cmd_sub = self.create_subscription(String, 'claw_cmd', self.claw_cmd_callback, 10)
            self.position_pub = self.create_publisher(Float64, 'position', 10)

            self.declare_parameter('in1_pin', 21)
            self.declare_parameter('in2_pin', 20)
            self.declare_parameter('en_pin', 6)

            self.declare_parameter('min_claw_value', 0.0)
            self.declare_parameter('max_claw_value', 5.0)

            self.IN1 = self.get_parameter('in1_pin').get_parameter_value().integer_value
            self.IN2 = self.get_parameter('in2_pin').get_parameter_value().integer_value
            self.EN = self.get_parameter('en_pin').get_parameter_value().integer_value
            self.MIN_VALUE = self.get_parameter('min_claw_value').get_parameter_value().double_value
            self.MAX_VALUE = self.get_parameter('max_claw_value').get_parameter_value().double_value

            timer_period = 0.1  # seconds
            self.timer = self.create_timer(timer_period, self.timer_position_pub)
            self.position_msg = Float64()

            self.init_hardware()
            self.get_logger().info('claw_controller has been started!')


        except Exception as e:
            self.get_logger().error(f'Error in ClawController initialization: {str(e)}')
            raise

    def init_hardware(self):
        self.get_logger().info('Initializing hardware components...')
        try:
            self.motor = L298N(en=self.EN, in1=self.IN1, in2=self.IN2)
            self.motor.set_duty_cycle(100)

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
            elif msg.data == 'stop':
                self.move_claw(0)
            else:
                self.get_logger().warn(f'Received invalid claw command: {msg.data}')
            
            self.get_logger().debug(f'Processed claw command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error in claw_cmd_callback: {str(e)}')

    def move_claw(self, direction):
        try:
            current_position = self.read_potentiometer()
            
            self.get_logger().info(f'Current position: {current_position}, Direction: {direction}')

            motor_sensor_treshold = 0.5
            
            if (direction > 0 and current_position > (self.MIN_VALUE + motor_sensor_treshold)) or \
               (direction < 0 and current_position < (self.MAX_VALUE - motor_sensor_treshold)):
                
                self.get_logger().info(f'Current position: {current_position}, Direction: {direction}')
                self.get_logger().info(f'current duty cycle: {self.motor.get_duty_cycle()}')
                if direction > 0:
                    self.motor.forward()
                    self.get_logger().info('Moving claw forward (open claw)')
                else:
                    self.motor.backward()
                    self.get_logger().info('Moving claw backward (close claw)')
            # else:
            #    self.motor.stop()
            
            self.motor.stop()
            self.get_logger().info('Claw movement stopped (at limit or no movement needed)')
                #self.get_logger().debug('Claw movement stopped (at limit or no movement needed)')
        except Exception as e:
            self.get_logger().error(f'Error in move_claw: {str(e)}')

    def read_potentiometer(self):
        
        try:
            voltage = self.adc.get_adc(0)
            self.get_logger().debug(f'Read potentiometer value: {voltage}')
            return voltage
        except Exception as e:
            self.get_logger().error(f'Error in read_potentiometer: {str(e)}')
            return -1
    
    def __del__(self):
        self.get_logger().info('Node is being destroyed')
        self.motor.cleanup()
        self.get_logger().info("claw motor's GPIO cleaneup")

def main(args=None):
    rclpy.init(args=args)
    try:
        claw_controller_node = ClawController()
        rclpy.spin(claw_controller_node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
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
