#!/usr/bin/env python

'''
This program was written by Chibuike Okpaluba and Marlon Gwira.

For more information please contact <##>
Thank you.

Copyright 2017 Middlesex
'''

## ******** Please Note *************** ##
#  Unless otherwise stated all units are specified in meters

import math
import time
import md25
import RPi.GPIO as GPIO
import generic_functions

class RobotObject(object):
	def __init__(self, robot_name="vima_worldskills", start_pin=4, wheel_diameter=0.1, distance_btw_wheels=0.065, encoder_counts_per_rev=360.0):
		self.start_pin       = start_pin
		self.start_btn_state = False

		# Define robot parameters
		self.wheel_diameter         = wheel_diameter
		self.distance_btw_wheels    = distance_btw_wheels
		self.encoder_counts_per_rev = encoder_counts_per_rev

		# Define constant to convert encoder counts to meters
		self.encoder_counts_to_meters_multiplier = (self.wheel_diameter * math.pi) / self.encoder_counts_per_rev

		# Define variables for speed and distance control
		self.prev_left_displacement   = 0.0
		self.prev_right_displacement  = 0.0
		self.curr_left_displacement   = 0.0
		self.curr_right_displacement  = 0.0

		# Motor power to be set
		self.left_command_wheel_power  = 0.0
		self.right_command_wheel_power = 0.0

		# Target wheel speed
		self.target_left_speed  = 0.0
		self.target_right_speed = 0.0

		# Actual wheel speed
		self.left_wheel_speed  = 0.0
		self.right_wheel_speed = 0.0
		
		self.last_update_time  = time.time()

		# Setup pull to start pin
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.start_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.add_event_detect(self.start_pin, GPIO.RISING)
		GPIO.add_event_callback(self.start_pin, self.start_btn_callback)

		# Setup md25
		self.wheels_controller = md25.MD25(0x58)

	def __encoder_counts_to_meters(self, counts):
		return counts * self.encoder_counts_to_meters_multiplier;

	def __update_wheel_data(self):
		# Ensure that the data is valid
		data = None; time_now = 0.0
		try:
			data = self.wheels_controller.get_encoder_counts()
			time_now = time.time() # Get time
			print "Encoder Counts: {0}  @ {1}s".format(data, time_now)
		except Exception as e:
			print "Error[{0}] when reading encoder counts".format(e)
			return

		# Get the difference in time in seconds
		time_delta = time_now - self.last_update_time
		
		# Get wheel distance
		# TODO: Prevent encoder reading overflow errors
		self.prev_left_displacement   = self.curr_left_displacement
		self.prev_right_displacement  = self.curr_right_displacement
		self.curr_left_displacement   = float( self.__encoder_counts_to_meters(data['left_displacement']) )
		self.curr_right_displacement  = float( self.__encoder_counts_to_meters(data['right_displacement']) )

		self.left_wheel_speed  = (self.curr_left_displacement - self.prev_left_displacement) / time_delta
		self.right_wheel_speed = (self.curr_right_displacement - self.prev_right_displacement) / time_delta

		# Update the time variable
		self.last_update_time = time_now

		# Update motor power
		self.__update_command_wheel_speed()

	def __update_command_wheel_speed(self, amplitude=0.035):
		# Calculate wheel speed error
		left_speed_error  = self.target_left_speed  - self.left_wheel_speed
		right_speed_error = self.target_right_speed - self.right_wheel_speed

		# Calculate the new motor power
		self.left_command_wheel_power  = self.left_command_wheel_power  + (amplitude * left_speed_error)
		self.right_command_wheel_power = self.right_command_wheel_power + (amplitude * right_speed_error)

		# Constrain motor power
		self.left_command_wheel_power  = generic_functions.constrainf(self.left_command_wheel_power, -100.0, 100.0)
		self.right_command_wheel_power = generic_functions.constrainf(self.right_command_wheel_power, -100.0, 100.0)

		# Set the motor power
		return self.wheels_controller.set_wheel_speeds(self.left_command_wheel_power, self.right_command_wheel_power)

	def __reset_wheel_data(self):
		self.prev_left_displacement   = 0.0
		self.prev_right_displacement  = 0.0
		self.curr_left_displacement   = 0.0
		self.curr_right_displacement  = 0.0

		self.left_command_wheel_power  = 0.0
		self.right_command_wheel_power = 0.0

		self.target_left_speed  = 0.0
		self.target_right_speed = 0.0

		self.left_wheel_speed  = 0.0
		self.right_wheel_speed = 0.0

		self.wheels_controller.set_wheel_speeds(self.left_command_wheel_power, self.right_command_wheel_power)
		return self.wheels_controller.reset_encoders()

	def set_left_wheel_speed(self, speed):
		self.target_left_speed = speed
		self.__update_wheel_data()

	def set_right_wheel_speed(self, speed):
		self.target_right_speed = speed
		self.__update_wheel_data()

	def set_wheel_speeds(self, left_speed, right_speed):
		self.target_left_speed = left_speed
		self.target_right_speed = right_speed
		self.__update_wheel_data()

	def get_start_switch_state(self):
		self.start_btn_state = GPIO.input(self.start_pin)
		return self.start_btn_state

	def start_btn_callback(self, data):
		self.get_start_switch_state()
		print data," := =: ", type(data)

	def move_distance(self, target_displacement=4.0, speed=1.0):
		# reset wheel data
		self.__reset_wheel_data()

		# Start moving
		speed = abs(speed)
		sign = target_displacement/abs(target_displacement)
		speed = sign * speed
		self.set_wheel_speeds(speed, speed)

		current_displacement = 0.0
		
		## -- DEBUG -- 
		#print "Dist: {0}m     Speed: ({1}, {2})m/s".format(current_displacement, self.left_wheel_speed, self.right_wheel_speed)
		#print "({0} - {1}      {2}m/s".format(abs(current_displacement), abs(target_displacement), speed)
		## -- DEBUG -- 
		
		while abs(current_displacement) - abs(target_displacement) < 0:
			self.__update_wheel_data()
			current_displacement = (self.curr_left_displacement + self.curr_right_displacement) / 2.0
			print "Dist: {0}m    Wheels: ({3}, {4})m     Speed: ({1}, {2})m/s".format(current_displacement, self.left_wheel_speed, self.right_wheel_speed, self.curr_left_displacement, self.curr_right_displacement)

		return self.wheels_controller.stop_wheels()

	def run(self):
		pass


if __name__ == "__main__":
	my_robot = RobotObject()
	print "Moving a distance"
	my_robot.move_distance(5.0, 1.0)
	my_robot.move_distance(-5.0, 1.0)