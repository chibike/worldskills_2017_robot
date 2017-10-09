#!/usr/bin/env python

def constrainf(x, x_min, x_max):
    '''Returns returns x if x is within x_min and x_max else returns x_min or x_max'''
    return min(max(x, x_min), x_max)

def mapf(x, old_min, old_max, new_min, new_max):
    '''Re-maps a number from one range to another.
    That is, a value of fromLow would get mapped to toLow,
    a value of fromHigh to toHigh, values in-between
    to values in-between, etc.'''

    return (x - old_min) * (new_max - new_min) / (old_max - old_min) + new_min