#!/usr/bin/env python

import smbus
import time

class I2cObject(object):
    def __init__(self, address):
        '''Initializes this object'''
        self.default_address = address
        self.bus = smbus.SMBus(1)

    def write(self, register, data):
        '''Write an array/list or int to an I2C device'''
        if isinstance(data, list) or isinstance(data, tuple):
        	self.bus.write_i2c_block_data(self.default_address, register, data)
        else:
        	self.bus.write_byte_data(self.default_address, register, int(data))

    def read(self, register, howmany):
        '''Returns an array containing [howmany] data elements read from selected register'''
        data = []
        for i in xrange(howmany):
            data.append(self.bus.read_byte_data(self.default_address, register+i))
        return data

    def read_as_uint(self, register, howmany):
        '''Returns the content of the register->register+howmany as an unsigned int'''
        return self.concatenate(self.read(register, howmany))

    def read_as_sint(self, register, howmany):
        '''Returns the content of the regiser->register+howmany as a signed int'''
        data = self.read(register, howmany)
        hex_string = "0x"
        for i in data:
            hex_string = hex_string + hex(i)[2:]
        return self.get_int(hex_string)

    def get_int(self, hex_string):
        '''Converts a hexstring to a signed interger using two's complement'''
        binary_string = bin(eval(hex_string))[2:]
        binary_string = "0" * (len(hex_string[2:])*4 - len(binary_string)) + binary_string
        if binary_string[0] == '0':
            return eval(hex_string)
        temp_str = '0b'

        # Reverse two's complement
        for i in binary_string:
            if i == '1':
                temp_str += '0'
            else:
                temp_str += '1'
        return -1*(eval(temp_str))

    def concatenate(self, data):
        '''Returns an list of bytes as a single bit string'''
        if isinstance(data, list) or isinstance(data, tuple):
            temp_str = "0x"
            for i in data:
                temp_str = temp_str + hex(i)[2:]
            return eval(temp_str)
        else:
            return data


if __name__ == '__main__':
    i2c_device = I2cObject(0x58)
    print "left_encoder = {0}".format(i2c_device.read_as_sint(2, 4))
    print "right_encoder = {0}".format(i2c_device.read_as_sint(6, 4))