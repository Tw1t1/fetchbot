import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

# from grab_ball.adc_reader import ADCReader
from grab_ball.hardware import L298N, ADCReader


import RPi.GPIO as GPIO
import time


class ClawController(Node):

    def __init__(self):
        super().__init__('claw_controller')
        
        self.claw_cmd_sub = self.create_subscription(String, 'claw_cmd', self.claw_cmd_callback, 10)
        
        self.position_pub = self.create_publisher(Float64, 'position', 10)

        self.create_timer(0.1, self.timer_position_pub)
        self.initialize_parameters()
        self.init_hardware()
        
        self.current_position = -1.0

        self.get_logger().info('claw_controller has been started!')

    def initialize_parameters(self):
        self.declare_parameter('in1_pin', 21)
        self.declare_parameter('in2_pin', 20)
        self.declare_parameter('min_sensor_value', 0.0)
        self.declare_parameter('max_sensor_value', 5.0)
        self.declare_parameter('claw_sensor_threshold', 10)
        self.IN1 = self.get_parameter('in1_pin').get_parameter_value().integer_value
        self.IN2 = self.get_parameter('in2_pin').get_parameter_value().integer_value
        self.MIN_SENSOR_VALUE = self.get_parameter('min_sensor_value').get_parameter_value().double_value
        self.MAX_SENSOR_VALUE = self.get_parameter('max_sensor_value').get_parameter_value().double_value
        self.claw_sensor_threshold = self.get_parameter('claw_sensor_threshold').get_parameter_value().integer_value

    def init_hardware(self):
        try:
            self.motor = L298N(in1=self.IN1, in2=self.IN2)
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
        msg = Float64()
        value = self.read_potentiometer() # read raw data in range [0-5]
        position = (value / self.MAX_SENSOR_VALUE) * 100.0 # convert data to % 
        msg.data = position
        self.position_pub.publish(msg)
        self.current_position = position


    def claw_cmd_callback(self, msg):
        try:
            msg = msg.data
            if (self.current_position > self.claw_sensor_threshold) or \
                (self.current_position < 100.0 - self.claw_sensor_threshold):
                
                if msg == 'open':
                    self.motor.forward()
                elif msg == 'close':
                    self.motor.backward()
                elif msg == 'stop':
                    self.motor.stop()
                else:
                    self.get_logger().warn(f'Received invalid claw command: {msg.data}')
            else:
                self.motor.stop()
                self.get_logger().info('self.motor.stop()')
        except Exception as e:
            self.get_logger().error(f'Error in claw_cmd_callback: {str(e)}')


    def read_potentiometer(self):
        try:
            voltage = self.adc.get_adc(0)
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
    claw_controller_node = ClawController()
    try:
        rclpy.spin(claw_controller_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        claw_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()