#!/usr/bin/env python

import smbus
import time

class I2cObject(object):
    def __init__(self, address):
        self.default_address = address
        self.bus = smbus.SMBus(1)

    def write(self, register, data):
        if isinstance(data, type(list)) or isinstance(data, type(tuple)):
        	pass
        else:
        	pass

    def read(self, register, howmany):
        data = []
        for i in xrange(howmany):
            data.append(self.bus.read_byte_data(self.default_address, register+i))
        return data


if __name__ == '__main__':
    i2c_device = I2cObject(0x58)
    print i2c_device.read(2, 4)


# bus = smbus.SMBus(0)
# address = 0x60
# bus.read_byte_data(address, howmany)
# bus.write_byte_data(address, register, data)
# bus.write_quick(address)
# bus.read_byte(address)
# bus.write_byte(address, data)
