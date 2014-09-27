#! /usr/bin/python

"""
receive_samples.py

By Paul Malmsten, 2010
pmalmsten@gmail.com

This example continuously reads the serial port and processes IO data
received from a remote XBee.
"""

from xbee import XBee
import serial
import struct
import time
import logging
from logging import handlers
from collections import deque


logging.basicConfig(level=logging.INFO)
formatter = logging.Formatter('%(asctime)s;%(message)s')            
logger = logging.getLogger("receive_samples")
logger.propagate = False
mainfh = handlers.RotatingFileHandler('main.log',maxBytes=50000, backupCount=5)
mainfh.setFormatter(formatter)
logger.addHandler(mainfh)

last_status = "ND"

dataloggers_dict={}

subscribed_nodes={}

icheck = 1
ilastcheck = 0

message_dict = {}

PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200 
scale_fact = 3.9
scale_adxl = 200
MB1 = 1048576
# Open serial port
ser = serial.Serial(PORT, BAUD_RATE)

def mapf( x, in_min, in_max, scalef):    
    return float((x - in_min) * 2.0 * scalef )/ float((in_max - in_min)) - float(scalef);

def help():
    print "commands are:"    
    print "\tstart"  
    print "\tstop"
    print "\tcheck"
    print "\tget"
    print "\tlog"
    print "\tnodes"
    print "\tmsg"
    print "\texit"

def log_data(received_data,i_source,datalog):
    long_millis = long(ord(received_data[26])<<24) + long(ord(received_data[27])<<16) + long(ord(received_data[28])<<8) + long(ord(received_data[29]))
    # 9DOF Accel
    str_ax = received_data[3] + received_data[2]
    iax = struct.unpack( "h",str_ax)[0] 
    fax = (float(iax)*scale_fact)/1000
    str_ay = received_data[5] + received_data[4]
    iay = struct.unpack( "h",str_ay)[0]
    fay = (float(iay)*scale_fact)/1000
    str_az = received_data[7] + received_data[6]
    iaz = struct.unpack( "h",str_az)[0]
    faz = (float(iaz)*scale_fact)/1000
    # 9DOF GYRO
    str_gx = received_data[9] + received_data[8]
    igx = struct.unpack( "h",str_gx)[0] 
    str_gy = received_data[11] + received_data[10]
    igy = struct.unpack( "h",str_gy)[0]
    str_gz = received_data[13] + received_data[12]
    igz = struct.unpack( "h",str_gz)[0]
    # 9DOF MAG
    str_mx = received_data[15] + received_data[14]
    imx = struct.unpack( "h",str_mx)[0] 
    str_my = received_data[17] + received_data[16]
    imy = struct.unpack( "h",str_my)[0]
    str_mz = received_data[19] + received_data[18]
    imz = struct.unpack( "h",str_mz)[0]
    # ACC ADXL377
    str_adxlx = received_data[21] + received_data[20]
    iadxlx = struct.unpack( "h",str_adxlx)[0]
    fadxlx =  mapf( iadxlx,0, 1023, scale_adxl)
    str_adxly = received_data[23] + received_data[22]
    iadxly = struct.unpack( "h",str_adxly)[0]
    fadxly = mapf( iadxly,0, 1023, scale_adxl)
    str_adxlz = received_data[25] + received_data[24]
    iadxlz = struct.unpack( "h",str_adxlz)[0]
    fadxlz = mapf( iadxlz,0, 1023, scale_adxl)
    datalog.info( "%d;%ld;%f;%f;%f;%d;%d;%d;%d;%d;%d;%f;%f;%f" %( i_source,long_millis,fax,fay,faz ,-1*igx,-1*igy,-1*igz,imx,-1*imy,-1*imz, fadxlx,fadxly,fadxlz ))


def message_received(response):
    i_source = 0
    received_data = response['rf_data']
    source_addr = response['source_addr']
    if len(source_addr) == 2:
        str_source = source_addr[1] + source_addr[0]
        i_source = struct.unpack( "h",str_source)[0]
        if i_source in dataloggers_dict:
            datalog = dataloggers_dict[i_source]
        else:
            sLogName = "data_%02d" % i_source
            datalog = logging.getLogger(sLogName)
            datalog.propagate = False
            datafh = handlers.RotatingFileHandler(sLogName+'.log',maxBytes=MB1*5, backupCount=10)
            datafh.setFormatter(formatter)
            datalog.addHandler(datafh)
            dataloggers_dict[i_source] = datalog
    if len(received_data) == 30 and received_data[0:2] == "ST":
        log_data(received_data,i_source,datalog)
    elif len(received_data) == 10 and received_data[0:2] == "CH":
        long_status = long(ord(received_data[2])<<24) + long(ord(received_data[3])<<16) + long(ord(received_data[4])<<8) + long(ord(received_data[5]))
        long_millis = long(ord(received_data[6])<<24) + long(ord(received_data[7])<<16) + long(ord(received_data[8])<<8) + long(ord(received_data[9]))
        print "\nreceived ack of %02d from %02d, at %ld status is %ld\n" % (ilastcheck, i_source,long_millis, long_status)
        logger.info("received ack of %02d from %02d, at %ld status is %ld" % (ilastcheck, i_source,long_millis, long_status))
        subscribed_nodes[i_source] = (struct.pack('>h', i_source), long_status)
    elif len(received_data) == 10 and received_data[0:2] == "SU":
        long_status = long(ord(received_data[2])<<24) + long(ord(received_data[3])<<16) + long(ord(received_data[4])<<8) + long(ord(received_data[5]))
        long_millis = long(ord(received_data[6])<<24) + long(ord(received_data[7])<<16) + long(ord(received_data[8])<<8) + long(ord(received_data[9]))
        subscribed_nodes[i_source] = (struct.pack('>h', i_source), long_status)
        print "\nreceived subscription from %02d, at %ld status is %ld\n" % (i_source,long_millis, long_status)
        logger.info("received subscription from %02d, at %ld status is %ld" % (i_source,long_millis, long_status))
        if i_source in message_dict:
            mess, addr = message_dict[i_source]
            xbee.tx(dest_addr=addr,data=mess)
            message_dict.pop(i_source)
            print " '%s' sent to %02d" % (mess, i_source)
            logger.info( " '%s' sent to %02d" % (mess, i_source))
            time.sleep(1)
    else:
        print "something wrong"

#xbee.tx(dest_addr='\x00\x01', data='Hello World') broadcast XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
# dest_addr_long='\x00\x00\x00\x00\x00\x00\xFF\xFF', dest_addr='\xFF\xFE'
# xbee.tx_long_addr(dest_addr='\x00\x00\x00\x00\x00\x00\xFF\xFF',data='S')
# Create API object
xbee = XBee(ser,callback=message_received, escaped=True)
# Continuously read and print packets
while True:
    try:
        reply = raw_input('Enter a command (h for help):')
        if reply == 'h' or 'help':
            logger.info("User enters HELP command")
            help()
        if reply == 'exit':
            last_status = reply
            logger.info("User enters exit command")
            break
        if reply == 'check':
            logger.info("User enters CHECK command number %02d" % icheck )
            xbee.tx_long_addr(dest_addr='\x00\x00\x00\x00\x00\x00\xFF\xFF',data='C%02d' % icheck)
            ilastcheck = icheck
            icheck += 1
        if reply == 'start':
            last_status = reply
            logger.info("User enters START command")
            xbee.tx_long_addr(dest_addr='\x00\x00\x00\x00\x00\x00\xFF\xFF',data='S')
            """
            for key in subscribed_nodes:
                addr, status = subscribed_nodes[key]
                if status == 0:
                    xbee.tx(dest_addr=addr,data='S')
                    logger.info(" 'S' sent to %02d" % key)
                    print " 'S' sent to %02d" % key
                    time.sleep(1)"""
        if reply == 'stop':
            last_status = reply
            logger.info("User enters STOP command")
            for key in subscribed_nodes:
                addr, status = subscribed_nodes[key]
                if status != 0:
                    message_dict[key] = ('T',addr)
                    print "message 'T' added to %02d" % key
                    logger.info("message 'T' added to %02d" % key)
                #xbee.tx(dest_addr=addr,data='T')
                #logger.info(" 'T' sent to %02d" % key)
                #print " 'T' sent to %02d" % key
                #time.sleep(2)
        if reply == 'log':
            logger.info("User enters LOG command")
            for key in subscribed_nodes:
                addr, status = subscribed_nodes[key]
                xbee.tx(dest_addr=addr,data='L')
                logger.info(" 'L' sent to %02d" % key)
                print " 'L' sent to %02d" % key
                time.sleep(1)
            #xbee.tx_long_addr(dest_addr='\x00\x00\x00\x00\x00\x00\xFF\xFF',data='L')
        if reply == 'get':
            logger.info("User enters READ command")
            xbee.tx_long_addr(dest_addr='\x00\x00\x00\x00\x00\x00\xFF\xFF',data='R')
        if reply == 'nodes':
            for node in subscribed_nodes:
                addr, status = subscribed_nodes[node]
                print "Node %02d subscribed, status %d" % (node,status )
                logger.info("Node %02d subscribed, status %d" % (node,status ) )
        if reply == 'msg':
            logger.info("User enters MSG command")
            for key in message_dict:
                sms , addr = message_dict[key]
                print "msg for %02d with addr %s is %s " % (key,addr,sms)
    except KeyboardInterrupt:
        logger.info("User enters ^C on the main loop")
        print "\nexiting main loop\n"
        break
xbee.halt()
ser.close()
