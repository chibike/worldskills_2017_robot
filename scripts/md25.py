#!/usr/bin/env python

'''
This program was written by Chibuike Okpaluba and Marlon Gwira.

For more information please contact <##>
Thank you.

Copyright 2017 Middlesex
'''

import time
import i2c_wrapper
import generic_functions

class MD25(object):
    def __init__(self, address=0x58):
        self.default_address = address
        self.i2c_object      = i2c_wrapper.I2cObject(address)

        # Initialize variables to track changes in encoder counts
        self.motor_direction_left     = 0
        self.motor_direction_right    = 0

        self.last_encoder_count_left  = 0
        self.last_encoder_count_right = 0

        self.displacement_left        = 0
        self.displacement_right       = 0

        # Set the mode of the controller
        self.set_mode(0)

        # Set wheel accerelation rate to 10
        self.set_acceleration_rate(10)

        # Set wheel speed to zero
        self.set_wheel_speeds(0,0)

        # Enable speed regulation
        self.enable_speed_regulation(True)

        # Disable timeout
        self.enable_timeout_after_2s(True)

        # Reset encoders
        self.reset_encoders()

    def __translate_value(self, value, low_threshold, high_threshold):
        '''Returns the right value based on the current mode'''
        self.get_mode()  # Gets the mode of the MD25
        value = generic_functions.constrainf(value, low_threshold, high_threshold)

        if self.mode == 0 or self.mode == 2:
            value = generic_functions.mapf(value, low_threshold, high_threshold, 0, 255)
        elif self.mode == 1 or self.mode == 3:
            value = generic_functions.mapf(value, low_threshold, high_threshold, -128, 127)
        else:
            raise ValueError("Invalid mode({0}) detected.".format(mode))

        return int(value)

    def set_mode(self, mode):
        '''Sets the mode of the controller'''
        # Ensure that the mode is between the acceptable range
        self.mode = generic_functions.constrainf(mode, 0, 3)
        self.i2c_object.write(15, self.mode)

    def get_mode(self):
        '''Returns the mode of the controller'''
        self.mode = self.i2c_object.read_as_uint(15, 1)
        return self.mode

    def stop_wheels(self):
        '''Stops the wheels'''
        self.motor_direction_left  = 0
        self.motor_direction_right = 0
        self.set_wheel_speeds(0,0)

    def set_wheel_speeds(self, left_speed, right_speed):
        '''Sets the speed of both wheels to the speed'''
        self.get_mode()

        self.left_speed  = generic_functions.constrainf(left_speed  + 1, -100, 100)
        self.right_speed = generic_functions.constrainf(right_speed + 1, -100, 100)

        self.motor_direction_left  = self.left_speed  / abs(self.left_speed)
        self.motor_direction_right = self.right_speed / abs(self.right_speed)

        if (self.mode == 0) or (self.mode == 1):
            left_value = self.__translate_value(self.left_speed, -100, 100)
            right_value = self.__translate_value(self.right_speed, -100, 100)
            self.i2c_object.write(0, left_value)
            self.i2c_object.write(1, right_value)
        elif (self.mode == 2) or (self.mode == 3):
            if self.left_speed != self.right_speed:
                raise ValueError("Both wheel speeds({1}, {2}) must be the same in this mode({0})".format(self.mode, self.left_speed, self.right_speed))
            else:
                value = self.__translate_value(self.left_speed, -100, 100)
                self.i2c_object.write(0, value)
        else:
            raise ValueError("Invalid mode({0}) detected".format(self.mode))

    def get_wheel_speeds(self):
        '''Returns the speed of the wheels'''
        left_speed_value_from_md25 = 0
        right_speed_value_from_md25 = 0

        if self.mode == 0:
            left_speed_value_from_md25 = self.i2c_object.read_as_uint(0, 1)
            left_speed_value_from_md25 = generic_functions.mapf(left_speed_value_from_md25, 0, 255, -100, 100)

            right_speed_value_from_md25 = self.i2c_object.read_as_uint(1, 1)
            right_speed_value_from_md25 = generic_functions.mapf(right_speed_value_from_md25, 0, 255, -100, 100)
        elif self.mode == 1:
            # TODO: fix sign errors
            left_speed_value_from_md25 = self.i2c_object.read_as_sint(0, 1)
            left_speed_value_from_md25 = generic_functions.mapf(left_speed_value_from_md25, -128, 127, -100, 100)

            right_speed_value_from_md25 = self.i2c_object.read_as_sint(1, 1)
            right_speed_value_from_md25 = generic_functions.mapf(right_speed_value_from_md25, -128, 127, -100, 100)
        elif self.mode == 2:
            left_speed_value_from_md25 = self.i2c_object.read_as_uint(0, 1)
            left_speed_value_from_md25 = generic_functions.mapf(left_speed_value_from_md25, 0, 255, -100, 100)

            right_speed_value_from_md25 = left_speed_value_from_md25
        elif self.mode == 3:
            # TODO: fix sign errors
            left_speed_value_from_md25 = self.i2c_object.read_as_sint(0, 1)
            left_speed_value_from_md25 = generic_functions.mapf(left_speed_value_from_md25, -128, 127, -100, 100)

            right_speed_value_from_md25 = left_speed_value_from_md25
        else:
            raise ValueError("Invalid mode({0}) detected".format(self.mode))

        wheel_speeds = dict()
        wheel_speeds["left_speed"] = (left_speed_value_from_md25, self.left_speed)
        wheel_speeds["right_speed"] = (right_speed_value_from_md25, self.right_speed)

        return wheel_speeds

    def get_encoder_counts(self):
        '''Returns the encoder counts for the wheels'''
        # Create a dictionary to store the values
        encoder_counts = dict()

        # Store the encoder counts for each wheel
        l = 0; r = 0; x = 1.0
        for i in xrange(int(x)):
            l += self.i2c_object.read_as_sint(2, 4)
            r += self.i2c_object.read_as_sint(6, 4)
        
        encoder_counts["left_count"]  = l/x
        encoder_counts["right_count"] = r/x

        # Calculate displacement for each wheel
        displacement_diff_left  = encoder_counts["left_count"]  - self.last_encoder_count_left
        displacement_diff_right = encoder_counts["right_count"] - self.last_encoder_count_right

        left_reading_is_reliable  = False
        right_reading_is_reliable = False

        try:
            if self.motor_direction_left == displacement_diff_left: # Handle zeros
                left_reading_is_reliable = True
            elif self.motor_direction_left == displacement_diff_left/abs(displacement_diff_left):
                left_reading_is_reliable = True
        except ZeroDivisionError as e:
            pass

        try:
            if  self.motor_direction_right == displacement_diff_right: # Handle zeros
                right_reading_is_reliable = True
            elif self.motor_direction_right == displacement_diff_right/abs(displacement_diff_right):
                right_reading_is_reliable = True
        except ZeroDivisionError as e:
            pass

        if left_reading_is_reliable or abs(self.last_encoder_count_left) >= 64095:
            # Determine displacement direction based on motor direction and increament accordingly
            self.displacement_left  += displacement_diff_left
            
            # Update the last encoder count variables
            self.last_encoder_count_left  = encoder_counts["left_count"]

        if right_reading_is_reliable or abs(self.last_encoder_count_right) >= 64095:
            # Determine displacement direction based on motor direction and increament accordingly
            self.displacement_right += displacement_diff_right
            
            # Update the last encoder count variables
            self.last_encoder_count_right = encoder_counts["right_count"]

        # Store displacements in dictionary
        encoder_counts["left_displacement"]  = self.displacement_left
        encoder_counts["right_displacement"] = self.displacement_right

        # Return dictionary
        return encoder_counts

    def reset_encoders(self):
        '''Resets the encoders'''
        self.motor_direction_left      = 0
        self.motor_direction_right     = 0
        self.last_encoder_count_left   = 0
        self.right_encoder_count_right = 0
        self.left_displacement         = 0
        self.right_displacement        = 0

        self.i2c_object.write(16, 0x20)

    def get_input_voltage(self):
        '''Returns the input voltage'''
        input_voltage = self.i2c_object.read_as_uint(10, 1)/10.0
        return input_voltage

    def get_motor_currents(self):
        '''Returns the current of the motors'''
        motor_currents= dict()
        motor_currents["left_current"] = self.i2c_object.read_as_uint(11, 1)/10.0
        motor_currents["right_current"] = self.i2c_object.read_as_uint(12, 1)/10.0
        return motor_currents

    def get_software_version(self):
        '''Returns the software version of the md25'''
        return self.i2c_object.read_as_uint(13, 1)

    def set_acceleration_rate(self, rate):
        '''Sets the acceleration to one of 10 steps'''
        rate = generic_functions.constrainf(rate, 0, 10)
        self.i2c_object.write(14, rate)

    def get_acceleration_rate(self):
        '''Returns the acceleration rate'''
        rate = self.i2c_object.read_as_uint(14, 1)
        return rate

    def change_address(self, new_address_index):
        '''Sets the address of the md25 to the new address selected by the index'''
        possible_addresses = (0xB0, 0xB2, 0xB4, 0xB6, 0xB8, 0xBA, 0xBC, 0xBE)
        new_address_index = generic_functions.constrainf(new_address_index, 0, len(possible_addresses)-1)
        new_address = possible_addresses[new_address_index]
        
        # Data to send
        cmd = (0xA0, 0xAA, 0xA5, new_address)
        self.i2c_object.write(16, cmd)

        self.address = new_address
        return self.address

    def enable_speed_regulation(self, status=True):
        '''Enables/Disables automatic speed regulation (default)'''
        if status:
            self.i2c_object.write(16, 0x31)
        else:
            self.i2c_object.write(16, 0x30)

    def enable_timeout_after_2s(self, status=True):
        '''Enables/Disables timeout of motors after 2 seconds when no I2C comms(default)'''
        if status:
            self.i2c_object.write(16, 0x33)
        else:
            self.i2c_object.write(16, 0x32)


def delay(time_ms):
    '''Delays for the given milliseconds'''
    start_time = time.time()
    while time.time() - start_time < (time_ms/1000.0):
        pass

def run_overall_test_routine():
    my_md25 = MD25(0x58)

    my_md25.set_wheel_speeds(100, -100)
    delay(5000)
    print "speed {0}".format(my_md25.get_wheel_speeds())
    print "acceleration_rate {0}".format(my_md25.get_acceleration_rate())
    print "version {0}".format(my_md25.get_software_version())
    print "encoders {0}".format(my_md25.get_encoder_counts())
    print "voltage {0}".format(my_md25.get_input_voltage())
    print "current {0}".format(my_md25.get_motor_currents())

    my_md25.set_wheel_speeds(50, -50)
    delay(3000)

    print "Stopping wheels"
    my_md25.stop_wheels()

    print "speed {0}".format(my_md25.get_wheel_speeds())
    print "acceleration_rate {0}".format(my_md25.get_acceleration_rate())
    print "version {0}".format(my_md25.get_software_version())
    print "encoders {0}".format(my_md25.get_encoder_counts())
    print "voltage {0}".format(my_md25.get_input_voltage())
    print "current {0}".format(my_md25.get_motor_currents())

    delay(1000)
    my_md25.set_wheel_speeds(0,0)

def run_write_test_routine():
    my_md25 = MD25(0x58)

    for j in xrange(0, 4):
        print "Setting mode to {0}".format(j)
        my_md25.set_mode(j)
        delay(100)
        print "Mode is {0}".format(my_md25.get_mode())
        for i in xrange(-100, 101, 10):
            print "setting speed to {0} units".format(i)
            my_md25.set_wheel_speeds(i,i)
            delay(300)
            print "speed {0}".format(my_md25.get_wheel_speeds())
        print "\n\n\n\n"
    my_md25.set_mode(0)
    my_md25.set_wheel_speeds(0,0)

def run_odometry_test():
    my_md25 = MD25(0x58)
    my_md25.motor_direction_left  = -1.0
    my_md25.motor_direction_right = +1.0
    while True:
        print "Encoder counts {0}".format(my_md25.get_encoder_counts())
        time.sleep(0.1)

if __name__ == '__main__':
    #run_overall_test_routine()
    #run_write_test_routine()
    run_odometry_test()
