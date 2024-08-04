

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64 # TODO modify custom_interfaces

from hardware_librery.ADCReader import ADCReader
from hardware_librery.L298N import L298N

class ClawController(Node):

    def __init__(self):
        
        super().__init__('claw_controller')
        self.claw_cmd_sub = self.create_subscription(String, 'claw_cmd', self.claw_cmd_callback, 10)
        self.position_pub = self.create_publisher(Float64, 'position', 10)


        self.declare_parameter('in1_pin', 21)
        self.declare_parameter('in2_pin', 20)
    
        self.declare_parameter('min_claw_value', 0.0)
        self.declare_parameter('max_claw_value', 5.0)
        
        # L298N pins numbers
        self.IN1 = self.get_parameter('in1_pin').get_parameter_value().integer_value
        self.IN2 = self.get_parameter('in2_pin').get_parameter_value().integer_value

        # Potentiometer limits
        self.MIN_VALUE = self.get_parameter('min_claw_value').get_parameter_value().double_value
        self.MAX_VALUE = self.get_parameter('max_claw_value').get_parameter_value().double_value

        # Timer to publish claw position
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_position_pub)
        self.position_msg = Float64()

        # L298N motor driver setup
        self.motor = L298N(en=None, in1=self.IN1, in2=self.IN2) 

        # ADC setup, using for read Potentiometer values 
        self.adc = ADCReader()

    def timer_position_pub(self):
        self.position_msg.data = self.read_potentiometer()
        self.position_pub.publish(self.position_msg)


    def claw_cmd_callback(self, msg):
        
        if msg.data == 'open':
            self.move_claw(1)  # Move in positive direction
        elif msg.data == 'close':
            self.move_claw(-1)  # Move in negative direction
        
    def move_claw(self, direction):
        current_position = self.read_potentiometer()

        if (direction > 0 and current_position < self.MAX_VALUE) or \
           (direction < 0 and current_position > self.MIN_VALUE):
            # Set motor direction and speed
            if direction > 0:
                self.motor.forward()
            else:
                self.motor.backward()
            self.motor.set_duty_cycle(50)  # 50% duty cycle
        else:
            # Stop motor if at limit or direction == 0
            self.motor.stop()

    def read_potentiometer(self):
        # Read ADC value from channel 0
        with self.adc:  # Using the new context manager
            voltage = self.adc.get_adc(0)
        return voltage

    def __del__(self):
        del self.motor  # This will clean up GPIO pins used by L298N

def main(args=None):
    rclpy.init(args=args)
    claw_controller_node = ClawController()
    rclpy.spin(claw_controller_node)
    claw_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
