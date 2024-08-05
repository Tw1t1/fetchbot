#!/usr/bin/python3

import RPi.GPIO as GPIO
import time


# based on this guide https://www.pibits.net/code/ky-009-3-color-led-smd-module-raspberry-pi-example.php


class RGBLed:
	
	def __init__(self, pin_R, pin_G, pin_B):
		self.pins = {'pin_R':pin_R, 'pin_G':pin_G, 'pin_B':pin_B}
		
		GPIO.setmode(GPIO.BCM)       						# Numbers GPIOs by BCM numbering	
		GPIO.setwarnings(False)
    	
		for key in self.pins:
			GPIO.setup(self.pins[key], GPIO.OUT)    		# Set pins' mode is output
        
		self.pwm_R = GPIO.PWM(self.pins['pin_R'], 2000)  	# Set Frequency to 2KHz
		self.pwm_G = GPIO.PWM(self.pins['pin_G'], 2000)
		self.pwm_B = GPIO.PWM(self.pins['pin_B'], 5000)

		self.pwm_R.start(0)      							 # Initial duty Cycle = 0 (LEDs off)
		self.pwm_G.start(0)
		self.pwm_B.start(0)
        
	def map_value(self, x, in_min, in_max, out_min, out_max):
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def set_color(self, col):   # For example : col = 0x112233
		R_val = (col & 0xFF0000) >> 16
		G_val = (col & 0x00FF00) >> 8
		B_val = (col & 0x0000FF) >> 0

		R_val = self.map_value(R_val, 0, 255, 0, 100)
		G_val = self.map_value(G_val, 0, 255, 0, 100)
		B_val = self.map_value(B_val, 0, 255, 0, 100)

		self.pwm_R.ChangeDutyCycle(R_val)     # Change duty cycle
		self.pwm_G.ChangeDutyCycle(G_val)
		self.pwm_B.ChangeDutyCycle(B_val)


	def set_color_rgb(self, R_val, G_val, B_val):
		R_val = self.map_value(R_val, 0, 255, 0, 100)
		G_val = self.map_value(G_val, 0, 255, 0, 100)
		B_val = self.map_value(B_val, 0, 255, 0, 100)

		self.pwm_R.ChangeDutyCycle(R_val)     # Change duty cycle
		self.pwm_G.ChangeDutyCycle(G_val)
		self.pwm_B.ChangeDutyCycle(B_val)
        
    
	def set_brightness(self, brightness):
		brightness = min(max(brightness, 0), 100)  # Clamp brightness between 0 and 100
		self.pwm_R.ChangeDutyCycle(brightness)
		self.pwm_G.ChangeDutyCycle(brightness)
		self.pwm_B.ChangeDutyCycle(brightness)

	def blink(self, col, times, on_duration=0.5, off_duration=0.5):
		for _ in range(times):
			self.set_color(col)
			time.sleep(on_duration)
			self.turn_off()
			time.sleep(off_duration)

	def cleanup(self):
		self.pwm_R.stop()
		self.pwm_G.stop()
		self.pwm_B.stop()
		GPIO.cleanup()

	def turn_off(self):
		self.set_color_rgb(0, 0, 0)
		
if __name__ == "__main__":
			#   red		green		blue	 	white	yellow	   purpule   turkiz
	colors = [0xFF0000, 0x00FF00, 0x0000FF,0xFFFFFF, 0xFFFF00, 0xFF00FF, 0x00FFFF]
	
	try:
		rgb_led = RGBLed(pin_R=19, pin_G=26, pin_B=13)
		delay = 1
		bright = 0
			
		while True:
			
			rgb_led.set_brightness(bright)
			
			rgb_led.set_color(0xFF0000)
			time.sleep(5)			
			
			bright +=10
			rgb_led.turn_off()
			time.sleep(1)
			
			#rgb_led.blink(col, 5, 0.3, 0.3)
			
			
			for col in colors:
				pass
				#rgb_led.set_color(col)
				#time.sleep(5)			
				#rgb_led.blink(col, 5, 0.3, 0.3)
				

	except KeyboardInterrupt:
		rgb_led.cleanup()
		print("\nExiting program")
		
		
		

