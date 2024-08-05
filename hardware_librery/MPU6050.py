#!/usr/bin/env python3

from mpu6050 import mpu6050

'''''''''''''''''''''
install before use:

1. sudo apt install python3-smbus
2. pip install mpu6050-raspberrypi

'''''''''''''''''''''

class MPU6050():

# add parameters shuch as accl_range
	def __init__(self, callback=None):
		
		self.mpu = mpu6050(0x68)
		
		self.callback = callback 
		## TODO implement callback event
		
	def read_sensor_data():
		acc = mpu.get_accel_data()
		gyro = mpu.get_gyro_data()
		temp = mpu.get_temp()
		
		return acc, gyro, temp
		
	def read_acceloremetr(self):
		return self.mpu.get_accel_data()

	def read_gyro(self):		
		return self.mpu.get_gyro_data()
	
	def read_temp(self):
		return self.mpu.get_temp()
		
	
