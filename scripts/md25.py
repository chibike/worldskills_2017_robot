#!/usr/bin/env python

'''
This program was written by Chibuike Okpaluba and Marlon Gwira.

For more information please contact <##>
Thank you.

Copyright 2017 Middlesex
'''

import i2c_wrapper
import generic_functions

class MD25(object):
    def __init__(self, address=0x58):
        self.default_address = address
        self.i2c_object = i2c_wrapper.I2cObject(address)

        # Set the mode of the controller
        self.set_mode(0)

        # Enable speed regulation
        self.enable_speed_regulation(True)

        # Disable timeout
        self.enable_timeout_after_2s(False)

        # Reset encoders
        self.reset_encoders()

    def __translate_value(self, value, low_threshold, high_threshold):
        '''Returns the right value based on the current mode'''
        self.get_mode()  # Gets the mode of the MD25
        value = generic_functions.constrainf(speed, low_threshold, high_threshold)

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
        self.set_wheel_speeds(0,0)

    def set_wheel_speeds(self, left_speed, right_speed):
        '''Sets the speed of both wheels to the speed'''
        self.get_mode()

        self.left_speed = generic_functions.constrainf(left_speed, -100, 100)
        self.right_speed = generic_functions.constrainf(right_speed, -100, 100)

        if (self.mode == 0) or (self.mode == 1):
            left_value = self.__translate_value(self.left_speed)
            right_value = self.__translate_value(self.right_speed)
            self.i2c_object.write(0, left_value)
            self.i2c_object.write(1, right_value)
        elif (self.mode == 2) or (self.mode == 3):
            if self.left_speed != self.right_speed:
                raise ValueError("Both wheel speeds({1}, {2}) must be the same in this mode({0})".format(self.mode, self.left_speed, self.right_speed))
            else:
                value = self.__translate_value(self.left_speed)
                self.i2c_object.write(0, left_value)
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
        encoder_counts = dict()
        encoder_counts["left_count"] = self.i2c_object.read_as_sint(2, 4)
        encoder_counts["right_count"] = self.i2c_object.read_as_sint(6, 4)
        return encoder_counts

    def reset_encoders(self):
        '''Resets the encoders'''
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



if __name__ == '__main__':
    my_md25 = MD25(0x58)
    wheel_speeds = my_md25.get_wheel_speeds()
    left_speed = wheel_speeds["left_speed"][1]
    right_speed = wheel_speeds["right_speed"][1]

    my_md25.set_wheel_speeds(100, -100)

