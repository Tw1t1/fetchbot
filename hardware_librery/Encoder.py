#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time


'''''''''''''''''''''

	# Class to monitor a rotary encoder and update a value.  You can either read the value when you need it, by calling getValue(), or
	# you can configure a callback which will be called whenever the value changes.
'''''''''''''''''''''


class Encoder:

    def __init__(self, leftPin, rightPin, callback=None):
        self.leftPin = leftPin
        self.rightPin = rightPin
        self.value = 0
        self.state = '00'
        self.direction = None
        self.callback = callback
        GPIO.setup(self.leftPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.rightPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.leftPin, GPIO.BOTH, callback=self.transitionOccurred)  
        GPIO.add_event_detect(self.rightPin, GPIO.BOTH, callback=self.transitionOccurred)  

    def transitionOccurred(self, channel):
        p1 = GPIO.input(self.leftPin)
        p2 = GPIO.input(self.rightPin)
        newState = "{}{}".format(p1, p2)

        if self.state == "00": # Resting position
            if newState == "01": # Turned right 1
                self.direction = "R"
            elif newState == "10": # Turned left 1
                self.direction = "L"

        elif self.state == "01": # R1 or L3 position
            if newState == "11": # Turned right 1
                self.direction = "R"
            elif newState == "00": # Turned left 1
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        elif self.state == "10": # R3 or L1
            if newState == "11": # Turned left 1
                self.direction = "L"
            elif newState == "00": # Turned right 1
                if self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        else: # self.state == "11"
            if newState == "01": # Turned left 1
                self.direction = "L"
            elif newState == "10": # Turned right 1
                self.direction = "R"
            elif newState == "00": # Skipped an intermediate 01 or 10 state, but if we know direction then a turn is complete
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                elif self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                
        self.state = newState

    def getValue(self):
        return self.value

# Optical Shaft Encoder (vex)
class OpticalShaftEncoder:
	def __init__(self, channel_a_pin, channel_b_pin, pulses_per_revolution=90, shaft_circumference=2.75*3.14, callback=None):
		self.channel_a_pin = channel_a_pin
		self.channel_b_pin = channel_b_pin
		self.pulses_per_revolution = pulses_per_revolution
		self.shaft_circumference = shaft_circumference

		self.position = 0
		self.prev_position = 0
		self.prev_time = time.time()
		self.callback = callback
		
		#self.setup()

	#def setup(self):
		#GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.channel_a_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(self.channel_b_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.add_event_detect(self.channel_a_pin, GPIO.BOTH, callback=self.update_position)
		GPIO.add_event_detect(self.channel_b_pin, GPIO.BOTH, callback=self.update_position)

	def read_state(self):
		state_a = GPIO.input(self.channel_a_pin)
		state_b = GPIO.input(self.channel_b_pin)
		return state_a, state_b

	def update_position(self, channel):
		state_a, state_b = self.read_state()

		if channel == self.channel_a_pin:
			if state_a == state_b:
				self.position += 1
			else:
				self.position -= 1
		elif channel == self.channel_b_pin:
			if state_a != state_b:
				self.position += 1
			else:
				self.position -= 1

	def read_position(self):
		return self.position

	def read_direction(self):
		return 1 if self.position > self.prev_position else -1 if self.position < self.prev_position else 0

	def read_speed(self):
		current_time = time.time()
		elapsed_time = current_time - self.prev_time

		if elapsed_time > 0:
			speed = (self.position - self.prev_position) / elapsed_time
			self.prev_position = self.position
			self.prev_time = current_time
			return speed
		else:
			return 0

	def read_distance(self):	
		# Calculate distance traveled based on the number of revolutions and shaft circumference
		revolutions = self.position / self.pulses_per_revolution
		distance = revolutions * self.shaft_circumference
		return distance

	def reset(self):
		self.position = 0
		self.prev_position = 0
		self.prev_time = time.time()

	def cleanup(self):
		GPIO.cleanup()
		
