#!/usr/bin/env python3

import RPi.GPIO as GPIO

MAX_DC_VAL = 100
PWM_FREQUENCY = 1200  # frequency in Hz, value set as vex motor controller 29

class L298N:
    """
    L298N motor driver class for controlling DC motors.
    """

    def __init__(self, en=None, in1=int, in2=int, defaultDutyCycle=30):
        """
        Initialize the L298N motor driver.

        :param en: Enable pin (ENA or ENB)
        :param in1: Input 1 pin (IN1 or IN3)
        :param in2: Input 2 pin (IN2 or IN4)
        :param defaultDutyCycle: Default duty cycle (0-100)
        """
        GPIO.setwarnings(False) 
        GPIO.setmode(GPIO.BCM)
        
        self.en = en
        self.in1 = in1
        self.in2 = in2
        
        self.defaultDutyCycle = max(0, min(defaultDutyCycle, 100))
        
        self.dutyCycleValue = 0
        
        GPIO.setup([self.in1, self.in2], GPIO.OUT)
        GPIO.output([self.in1, self.in2], GPIO.LOW)

        if self.en is not None:
            GPIO.setup(self.en, GPIO.OUT)
            self.pwm = GPIO.PWM(self.en, PWM_FREQUENCY)
            self.pwm.start(self.dutyCycleValue)

    def forward(self):
        """
        Set the motor to move forward.
        """
        if self.en is not None:
            self.pwm.ChangeDutyCycle(self.dutyCycleValue)
        
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        
    def backward(self):
        """
        Set the motor to move backward.
        """
        if self.en is not None:
            self.pwm.ChangeDutyCycle(self.dutyCycleValue)
        
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
    
    def set_duty_cycle(self, dc=100):
        """
        Set the duty cycle for PWM.

        :param dc: Duty cycle (0-100)
        """
        self.dutyCycleValue = max(0, min(dc, MAX_DC_VAL))
        
    def get_duty_cycle(self):
        """
        Get the current duty cycle.

        :return: Current duty cycle
        """
        return self.dutyCycleValue

    def stop(self):
        """
        Stop the motor.
        """
        GPIO.output([self.in1, self.in2], GPIO.LOW)
        if self.en is not None:
            self.pwm.ChangeDutyCycle(0)

    def set_pwm_frequency(self, frequency):
        """
        Set a new PWM frequency.

        :param frequency: New PWM frequency in Hz
        """
        if self.en is not None:
            self.pwm.ChangeFrequency(frequency)

    def cleanup(self):
        self.stop()
        if self.en is not None:
            GPIO.cleanup([self.en, self.in1, self.in2])
        else:
            GPIO.cleanup([self.in1, self.in2])

