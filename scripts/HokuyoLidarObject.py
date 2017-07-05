#!/usr/bin/env python

import time
import serial

class HokuyoLidarObject(object):
    def __init__(self, port, baud):
        self.serial = serial.Serial(port, baud, timeout=0.065)
        self.changeVersion()

    def isActive(self):
        if self.serial != None:
            try:
                return self.serial.isOpen()
            except:
                return False
        return False

    def write(self, data):
        if not self.isActive(): return
        self.serial.write(data + "\n")

    def readAll(self, timeout=-1):
        if not self.isActive(): return ""
        
        if timeout >= 0:
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if self.serial.inWaiting() > 0:
                    break
        else:
            while 1:
                if self.serial.inWaiting() > 0:
                    break
        return self.serial.readall()

    def changeVersion(self):
        if not self.isActive(): return
        self.write("SCIP2.0")
        return self.readAll()[9:11]

    def getSample(self, howmany=1, scan_interval=1, step=(0, 768), cluster_count=1 ):
        if not self.isActive(): return

        # Must be between 0 and 768
        step    = list(step)
        step[0] = int(max(0, min(step[0], 768)))
        step[1] = int(max(0, min(step[1], 768)))

        # Must be greater
        if step[1] < step[0]: return

        step[0]       = self._getFixedLength(str(step[0]), 4, '0', '')
        step[1]       = self._getFixedLength(str(step[1]), 4, '0', '')
        cluster_count = self._getFixedLength(str(int(cluster_count)), 2, '0', '')
        scan_interval = self._getFixedLength(str(int(scan_interval)), 1, '0', '')
        howmany       = self._getFixedLength(str(int(howmany)), 2, '0', '')
        command       = "MS" + step[0] + step[1] + cluster_count + scan_interval + howmany
        self.write(command)

        response   = self.readAll()
        status     = response[37:39]
        time_stamp =self._decodeTimeStamp(response[41:45])

        data = response[47:-6].split('\n'); measurments = []
        for d in data:
            if self._authenticateData(d):
                measurments += self._decode2CharacterCoding(d)
            else:
                measurments += [0]*32
        data_length = len(measurments)
        
        return {'data':measurments,'data_length':data_length,'time_stamp':time_stamp,'status':status}
        

    def close(self):
        if self.isActive():
            self.serial.close()

    def _authenticateData(self, data):
        auth_sum = data[-1]
        data_sum = 0
        for char in data[:-1]:
            data_sum += ord(char)

        data_sum = int(bin(data_sum)[2:][-6:], 2) + 48
        data_sum = chr(data_sum)
        return data_sum == auth_sum

    def _getFixedLength(self, string, length=0, prefix='0', suffix=''):
        while len(string) < length:
            string = prefix + string + suffix
        return string[0:length]

    def _decodeTimeStamp(self, time_stamp):
        time = ''
        for char in time_stamp:
            time += self._getFixedLength(bin(ord(char) - 48)[2:], 6, '0', '')
        return int(time, 2)

    def _decode2CharacterCoding(self, string):
        data = []
        for index in range(0, len(string)-2, 2):
            first_part = self._getFixedLength(bin(ord(string[index]) - 48)[2:], 6, '0', '')
            second_part = self._getFixedLength(bin(ord(string[index+1]) - 48)[2:], 6, '0', '')
            data.append(int(first_part + second_part, 2))
        return data

if __name__ == '__main__':
    device = HokuyoLidarObject('/dev/ttyUSB0', 115200);
    print "sample:",device.getSample()
    device.close()
