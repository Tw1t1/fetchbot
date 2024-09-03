#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Collision  # Replace with your actual package name
import RPi.GPIO as GPIO

class BumperNode(Node):
    def __init__(self):
        super().__init__('bumper')
        
        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        self.left_bumper_pin = 4
        self.right_bumper_pin = 18
        GPIO.setup(self.left_bumper_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_bumper_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Create publisher for the collision topic
        self.collision_pub = self.create_publisher(Collision, 'collision', 10)
        
        # Create a timer to check the sensors
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Define angles
        self.left_angle = 2.35
        self.right_angle = -2.35
        self.rear_angle = 3.14

        # Define the range
        self.right_range = 0.5
        self.left_range = 0.5

    def read_bumper_sensors(self):
        # Read the state of both bumper sensors
        # GPIO.input returns 0 if pressed (circuit closed), 1 if not pressed
        left_bumper_pressed = not GPIO.input(self.left_bumper_pin)
        right_bumper_pressed = not GPIO.input(self.right_bumper_pin)
        return left_bumper_pressed, right_bumper_pressed

    def calculate_collision_angle(self, left_pressed, right_pressed):
        if left_pressed and right_pressed:
            return self.rear_angle, min(self.left_range, self.right_range)
        elif left_pressed:
            return self.left_angle, self.left_range
        elif right_pressed:
            return self.right_angle, self.right_range
        else:
            return None  # No collision

    def timer_callback(self):
        left_pressed, right_pressed = self.read_bumper_sensors()
        collision_angle, collision_range = self.calculate_collision_angle(left_pressed, right_pressed)
        
        if collision_angle is not None:
            collision_msg = Collision()
            collision_msg.angle, collision_msg.range = float(collision_angle), float(collision_range)
            self.collision_pub.publish(collision_msg)

    def __del__(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    bumper_node = BumperNode()
    try:
        rclpy.spin(bumper_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            bumper_node.destroy_node()
            rclpy.shutdown()
            GPIO.cleanup()

if __name__ == '__main__':
    main()