#!/usr/bin/python

import sys
import serial
import io
import threading
import time
import os
import struct
from serial import Serial

sPort = "/dev/ttyATH0"
iBautRate = 115200

DATA_SYNC = 0x71
DATA_ACK = 0x7E
DATA_10B = 0x7F
DATA_16B = 0x7D
START_MSG = 0xAA
DATA_TYPE = 0x44
SYNC_TYPE = 0x53
END_MSG = 0x0A
lsm303Accel_MG_LSB = 0.012    #2g=0.001, 4g=0.002, 8g=0.004 or 16g=0.0125 mg per lsb
SENSORS_GRAVITY_STANDARD = 9.80665

class ImuSerial(Serial):
    def __init__(self, *args, **kwargs):
        #ensure that a reasonable timeout is set
        timeout = kwargs.get('timeout',0.1)
        if timeout < 0.01: timeout = 0.01
        kwargs['timeout'] = timeout
        Serial.__init__(self, *args, **kwargs)
        self.buf = ''
        self.lastMillis = 0
        self.messaggehandlers = {}
        self.isClosing= False;
    
    def add_message_handler(self,key, func):
        self.messaggehandlers[key] = func
    
    def send_message(self, strData):
        retBytes = self.write(chr(START_MSG))
        for c in strData:
            self.write(c)
        self.write(chr(END_MSG))
        
        return retBytes
    
    def available(self):
        if self.isClosing:
            return 0
        else:
            return self.inWaiting()
    
    def iterate(self):
        handlerKey, rec_array = self.readmessage()
        if len(rec_array) == 0 :
            pass
        elif handlerKey in self.messaggehandlers:
            try:
                _func = self.messaggehandlers[handlerKey]
                if callable(_func):
                    _func(rec_array)
            except ValueError:
                pass
        else:
            raise AssertionError("Wrong Key in dictionary: " + handlerKey)
            
    def exit(self):
        self.isClosing = True
        self.close()
    
    def readmessage(self, maxsize=None, timeout=1):
        """maxsize is ignored, timeout in seconds is the max time that is way for a complete message"""
        errLong = 999L
        self.lastMillis = 999L
        strOut="%ld;%ld;%ld;%ld;%ld;%ld;%ld" % (errLong,errLong,errLong,errLong,errLong,errLong,errLong)
        byte = self.read()
        if not byte:
            return "yyy;yyy;yyy;yyy;yyy;yyy;yyy"
        data = ord(byte)
        messaggehandler_key = 0
        received_data = []
        if data == START_MSG:
            data = ord(self.read())
            if data == DATA_TYPE:
                data = ord(self.read())
                messaggehandler_key = data
                data = ord(self.read())
                while data != END_MSG:
                    received_data.append(data)
                    data = ord(self.read())
        return messaggehandler_key, received_data


#parse data with 10 bytes (only acceleration)
def parse_data_message(received_data):
    if len(received_data) == 10: #16
        long_millis = long(received_data[0]<<24) + long(received_data[1]<<16) + long(received_data[2]<<8) + long(received_data[3])
        str_ax = chr(received_data[5]) + chr(received_data[4])
        str_ay = chr(received_data[7]) + chr(received_data[6])
        str_az = chr(received_data[9]) + chr(received_data[8])
        #str_gx = chr(received_data[11]) + chr(received_data[10])
        #str_gy = chr(received_data[13]) + chr(received_data[12])
        #str_gz = chr(received_data[15]) + chr(received_data[14]) 
        lax = struct.unpack( "h",str_ax)[0]
        lay = struct.unpack( "h",str_ay)[0]
        laz = struct.unpack( "h",str_az)[0]
        #lgx = struct.unpack( "h",str_gx)[0]
        #lgy = struct.unpack( "h",str_gy)[0]
        #lgz = struct.unpack( "h",str_gz)[0]
        accx = lsm303Accel_MG_LSB*SENSORS_GRAVITY_STANDARD*float(lax)/float(16)
        accy = lsm303Accel_MG_LSB*SENSORS_GRAVITY_STANDARD*float(lay)/float(16)
        accz = lsm303Accel_MG_LSB*SENSORS_GRAVITY_STANDARD*float(laz)/float(16)
        strOut="%ld;%f;%f;%f;%d;%d;%d" % (long_millis,accx,accy,accz,lax,lay,laz)
        print strOut
        return long_millis, strOut
    else:
        return 0, "xxx"

#parse data with 10 bytes (only acceleration)
def parse_data_6b_message(received_data):
    if len(received_data) == 6:
        long_millis = long(received_data[0]<<24) + long(received_data[1]<<16) + long(received_data[2]<<8) + long(received_data[3])
        str_int = chr(received_data[5]) + chr(received_data[4])
        intdata = struct.unpack( "h",str_int)[0]
        strOut="%ld;%d" % (long_millis,intdata)
        print strOut
        return long_millis, strOut
    else:
        return 0, "xxx"


#parse data with 16 bytes (accelerationa and gyriscope)
def parse_data_16b_message(received_data):
    if len(received_data) == 16: 
        long_millis = long(received_data[0]<<24) + long(received_data[1]<<16) + long(received_data[2]<<8) + long(received_data[3])
        str_ax = chr(received_data[5]) + chr(received_data[4])
        str_ay = chr(received_data[7]) + chr(received_data[6])
        str_az = chr(received_data[9]) + chr(received_data[8])
        str_gx = chr(received_data[11]) + chr(received_data[10])
        str_gy = chr(received_data[13]) + chr(received_data[12])
        str_gz = chr(received_data[15]) + chr(received_data[14]) 
        lax = struct.unpack( "h",str_ax)[0]
        lay = struct.unpack( "h",str_ay)[0]
        laz = struct.unpack( "h",str_az)[0]
        lgx = struct.unpack( "h",str_gx)[0]
        lgy = struct.unpack( "h",str_gy)[0]
        lgz = struct.unpack( "h",str_gz)[0]
        strOut="%ld;%d;%d;%d;%d;%d;%d" % (long_millis,lax,lay,laz,lgx,lgy,lgz)
        print strOut
        return long_millis, strOut
    else:
        return 0, "xxx"

class Iterator(threading.Thread):
    def __init__(self, serialComm):
        super(Iterator, self).__init__()
        self.serialComm = serialComm
    
    def run(self):
        while 1:
            try:
                while self.serialComm.available():
                    self.serialComm.iterate()
                    time.sleep(0.0005)
            except (AttributeError, serial.SerialException, OSError), e:
                # this way we can kill the thread by setting the board object
                # to None, or when the serial port is closed by board.exit()
                break
            except Exception, e:
                # catch 'error: Bad file descriptor'
                # iterate may be called while the serial port is being closed,
                # causing an "error: (9, 'Bad file descriptor')"
                if getattr(e, 'errno', None) == 9:
                    break
                try:
                    if e[0] == 9:
                        break
                except (TypeError, IndexError):
                    pass
                raise


        

if __name__=='__main__':
    s = ImuSerial(sPort, iBautRate)
    s.flushInput()
    s.flushOutput()
    #s.add_message_handler(DATA_10B,parse_data_message)
    s.add_message_handler(DATA_16B,parse_data_16b_message)
    s.add_message_handler(DATA_SYNC,parse_data_6b_message)
    s.add_message_handler(DATA_ACK,parse_data_6b_message)
    iCount = 1
    while True:
        try:
            while s.available():
                s.iterate()
                time.sleep(0.0005)
        except (AttributeError, serial.SerialException, OSError), e:
            print str(e)
        iCount += 1
