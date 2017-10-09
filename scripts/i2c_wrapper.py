#!/usr/bin/env python

import smbus
import time

class I2cObject(object):
    def __init__(self, address):
        self.default_address = address
        self.bus = smbus.SMBus(0)

    def write(self, register, data):
        pass

    def read(self, register, howmany):
        pass


# bus = smbus.SMBus(0)
# address = 0x60
# bus.read_byte_data(address, howmany)
# bus.write_byte_data(address, register, data)
# bus.write_quick(address)
# bus.read_byte(address)
# bus.write_byte(address, data)
