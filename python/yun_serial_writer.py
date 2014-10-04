#!/usr/bin/env python

# Copyright (c) Twisted Matrix Laboratories.
# See LICENSE for details.

"""
An example demonstrating how to send and receive UDP broadcast messages.

Every second, this application will send out a SYNC message with a unique ID.
It will respond to all SYNC messages with a PONG (including ones sent by
itself). You can tell how many copies of this script are running on the local
network by the number of "RECV PONG".

Run using twistd:

$ twistd -ny udp_mc_client_serial.py
"""
import Queue
import sys, os, glob
import serial, socket
from serial import Serial
from datetime import datetime
from twisted.application import internet, service
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import threads
from twisted.internet import reactor
import logging
from logging import handlers
import ConfigParser
import time
import threading
import struct
from collections import deque

config = ConfigParser.ConfigParser()
config.read('udp_mc_client.conf')

DATA_SYNC = 0x71
DATA_ACK = 0x7E
DATA_10B = 0x7F
DATA_26B = 0x7D
DATA_14BA = 0x7A
DATA_14BM = 0x7B
DATA_14BG = 0x7C
START_MSG = 0xAA
END_MSG = 0x0A
lsm303Accel_MG_LSB = 0.012          #2g=0.001, 4g=0.002, 8g=0.004 or 16g=0.0125 mg per lsb
lsm303Mag_Gauss_LSB_XY = 1100.0     #Varies with gain
lsm303Mag_Gauss_LSB_Z  = 980.0      #Varies with gain
SENSORS_GAUSS_TO_MICROTESLA = 100.0
SENSORS_GRAVITY_STANDARD = 9.80665
GYRO_SENSITIVITY_2000DPS = 0.070
SENSORS_DPS_TO_RADS = 0.017453293

class ImuSerial(Serial):
    def __init__(self, *args, **kwargs):
        #ensure that a reasonable timeout is set
        timeout = kwargs.get('timeout',0.1)
        if timeout < 0.01: timeout = 0.01
        kwargs['timeout'] = 0.01
        Serial.__init__(self, *args, **kwargs)
        self.buf = ''
        self.msgBytesRead = 0
        self.lastMillis = 0
        self.messaggehandlers = {}
        self.isClosing= False;
        self.dataqueue = Queue.Queue(0)
        self.parsingMessage = False
    
    def add_message_handler(self,key, func):
        self.messaggehandlers[key] = func
        logger.info("ImuSerial:add_message_handler %s with key %s" %(func.__name__, key))
    
    def send_message(self, strData):
        retBytes = self.write(chr(START_MSG))
        for c in strData:
            retBytes += self.write(c)
        retBytes += self.write(chr(END_MSG))
        return retBytes
    
    def available(self):
        if self.isClosing:
            return 0
        else:
            return self.inWaiting()
    
    def iterateNew(self):
        inputData = self.read()
        if self.parsingMessage:
            if inputData == chr(END_MSG):
                self.parsingMessage = False
                if len(self.buf) > 0:
                    # messaggehandlers
                    handlerKey = ord(self.buf[0])
                    rec_array = self.buf[1:]
                    if handlerKey in self.messaggehandlers:
                        try:
                            _func = self.messaggehandlers[handlerKey]
                            if callable(_func):
                                _func(rec_array)
                            else:
                                logger.error("iterateNew object {0} for key {1} not callable".format(str(_func), hex(handlerKey)))
                        except ValueError:
                            logger.error("iterateNew call to {0} with key {1} failed!".format(_func.__name__,  hex(handlerKey)))
                            pass
                    else:
                        logger.error("iterateNew key {0} not in messaggehandlers!".format( hex( handlerKey )))
                    self.buf = ""
                else:
                    logger.error("iterateNew Empty Buffer")
            else:                
                self.buf += inputData
                self.msgBytesRead += 1
        else:
            if inputData == chr(START_MSG):
                self.msgBytesRead = 0
                self.parsingMessage = True
    
    def iterate(self):
        handlerKey, rec_array = self.readmessage()
        if len(rec_array) == 0 :
            pass
        elif handlerKey in self.messaggehandlers:
            try:
                _func = self.messaggehandlers[handlerKey]
                if callable(_func):
                    _func(rec_array)
                else:
                    logger.error("object %s for key %s not callable" %(str(_func),handlerKey))
            except ValueError:
                logger.error("call to %s with key %s failed!" %(_func.__name__, handlerKey))
                pass
        else:
            logger.error("Something wrong for key %s" % handlerKey)
            pass
            
    def exit(self):
        self.isClosing = True
        self.close()
    
    def readmessage(self, maxsize=None, timeout=1):
        """maxsize is ignored, timeout in seconds is the max time that is way for a complete message"""
        received_data = []
        messaggehandler_key = 0
        try:
            byte = self.read()
            if not byte:
                return 0x00, received_data
            data1 = byte
            if data1 == chr(START_MSG):
                data3 = self.read()
                messaggehandler_key = ord(data3)
                if messaggehandler_key not in self.messaggehandlers:
                    logger.error("Something wrong in readmessage read data = %s" % chr(data1)+chr(data3))
                    return 0, []
                data = self.read()
                while data != chr(END_MSG):
                    received_data.append(ord(data))
                    data = self.read()
        except TypeError:
            logger.error("Something wrong in readmessage received_data = %s" % str(received_data))
        return messaggehandler_key, received_data


def am_i_alive():
    logger.info("Writer Alive")


def sendDataLog(sendAll=False):
    logger.info("Main:sendDataLog(%s)" % str(sendAll))
    copyed_size = 0L
    copyed_items = 0
    remote_folder = config.get('ssh', 'remote_folder')
    log_folder = config.get('logging', 'log_folder')
    dtnow = datetime.utcnow()
    strnow = dtnow.strftime('%Y%m%d%H%M%S')
    for file in glob.glob( os.path.join(log_folder,"*.*") if sendAll else os.path.join(log_folder,"*.log.*") ):
        logger.info("Main:sendDataLog processing %s" % file)
        statinfo = os.stat(file)
        remotefile = os.path.join(remote_folder, "%s_%s_%s" % (socket.gethostname(),strnow,os.path.basename(file)) )
        scpCommand = 'scp "%s" "%s@%s:%s" -i "/root/.ssh/id_rsa"' % (file, config.get('ssh', 'remote_username'),config.get('ssh', 'remote_ssh_host_ip'), remotefile)
        logger.info("Main:sendDataLog scpCommand %s " % scpCommand)
        retCode = os.system(scpCommand)
        if retCode==0:
            copyed_size += statinfo.st_size
            copyed_items += 1
            os.remove(file)
            logger.info("Main:sendDataLog file %s removed after scp copied to %s" % (file,config.get('ssh', 'remote_ssh_host_ip')))
        else:
            logger.error("Main:sendDataLog unable to scp file %s to %s" % (file,config.get('ssh', 'remote_ssh_host_ip')))
    return copyed_size , copyed_items


class NrsWriterClientProtocol(DatagramProtocol):
    noisy = False

    def __init__(self, controller, port):
        self.port = port
        self.inData = ""
        self.hostname = socket.gethostname()
        self.imuserial = ImuSerial(config.get('serial', 'port'), int(config.get('serial', 'baudrate')))
        self.imuserial.flushInput()
        self.imuserial.flushOutput()    
        self.imuserial.add_message_handler(DATA_26B, self.parse_data_26b_data_message)
        self.imuserial.add_message_handler(DATA_14BA, self.parse_data_14ba_data_message)
        self.imuserial.add_message_handler(DATA_14BM, self.parse_data_14bm_data_message)
        self.imuserial.add_message_handler(DATA_14BG, self.parse_data_14bg_data_message)
        self.imuserial.add_message_handler(DATA_SYNC, self.parse_data_6b_sync_message)
        self.imuserial.add_message_handler(DATA_ACK,self.parse_data_6b_ack_message)
        self.lastmillis = 0
        self.indata = "ND"
        self.istatus = -1
        self.localDateTime = "ND"
        self.continueLoop = True
              
    def startProtocol(self):
        self.transport.joinGroup(config.get('udp', 'multicast_address'))
        
    # qui deve salvare i file e spedirli via scp al server
    def sendLog(self,copyed_size, copyed_items):
        logMsg = "RET_LOG {0}:{1}".format(copyed_items,copyed_size)
        self.transport.write(logMsg, (config.get('udp', 'multicast_address'), self.port))
        logger.info("NrsWriterClientProtocol:sendLog SENDS %s" % logMsg)

    def datagramReceived(self, datagram, (host, port)):
        if datagram[:4] == "SYNC":
            self.indata = datagram[5:]
            logger.info("NrsWriterClientProtocol:datagramReceived %s from %s:%d" % (self.indata,host, port))
            dtnow = datetime.utcnow()
            self.localDateTime = dtnow.strftime('%Y;%m;%d;%H;%M;%S;%f')
            self.sendData("SYNC")
            sMsg = "%s;%ld;%d" % (self.indata,0,0)
            pongMsg = "OKSYNC " + sMsg
            self.transport.write(pongMsg, (host, port))
            # ricevuto il SYNC, chiedo i miei millis e li rimando al server
        if datagram[:5] == "START":
            self.imuserial.flushInput()
            self.imuserial.flushOutput() 
            self.sendData("STRT")
            logger.info("NrsWriterClientProtocol:datagramReceived %s from %s:%d" % (datagram,host, port))
            self.transport.write("START received", (host, port))
        if datagram[:4] == "STOP":
            self.sendData("STOP")
            logger.info("NrsWriterClientProtocol:datagramReceived %s from %s:%d" % (datagram,host, port))
            self.transport.write("STOP received", (host, port))
        if datagram[:6] == "STATUS":
            self.sendData("STAT")
            logger.info("NrsWriterClientProtocol:datagramReceived %s from %s:%d" % (datagram,host, port))
            self.transport.write("STATUS received", (host, port))
        if datagram[:6] == "GETLOG":
            copyed_size, copyed_items = sendDataLog()
            self.sendLog(copyed_size, copyed_items)
        if datagram[:5] == "EXIT":
            self.exitBoard()
            self.transport.write("Board Terminated", (host, port))

    def exitBoard(self):
        logger.info("NrsWriterClientProtocol::exitBoard")
        self.imuserial.exit()
    
    def setDataLogger(self,dtlogger):
        self.datalogger = dtlogger
        self.datalogger.info("============== STARTING ON %s ==============" %  config.get('main', 'hostname'))

    def sendData(self,strdata):
        retData = self.imuserial.send_message(strdata)
        logger.info("NrsWriterClientProtocol::sendData('%s') has sent %d bytes" % (strdata,retData))

    #parse data with 26 bytes (accelerationa and gyriscope)
    def parse_data_26b_data_message(self, received_data):
        if len(received_data) == 22: 
            long_millis = long(ord(received_data[0])<<24) + long(ord(received_data[1])<<16) + long(ord(received_data[2])<<8) + long(ord(received_data[3]))
            str_ax = received_data[5] + received_data[4]
            str_ay = received_data[7] + received_data[6]
            str_az = received_data[9] + received_data[8]
            str_mx = received_data[11] + received_data[10]
            str_my = received_data[13] + received_data[12]
            str_mz = received_data[15] + received_data[14] 
            str_gx = received_data[17] + received_data[16]
            str_gy = received_data[19] + received_data[18]
            str_gz = received_data[21] + received_data[20]
            lax = struct.unpack( "h",str_ax)[0]
            lay = struct.unpack( "h",str_ay)[0]
            laz = struct.unpack( "h",str_az)[0]
            lmx = struct.unpack( "h",str_mx)[0]
            lmy = struct.unpack( "h",str_my)[0]
            lmz = struct.unpack( "h",str_mz)[0]
            lgx = struct.unpack( "h",str_gx)[0]
            lgy = struct.unpack( "h",str_gy)[0]
            lgz = struct.unpack( "h",str_gz)[0]
            # ACCELEROMETRO
            accx = lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD * float(lax) / float(16)
            accy = lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD * float(lay) / float(16)
            accz = lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD * float(laz) / float(16)    
            # MAGNETOMETRO
            magx = float(lmx) / lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA
            magy = float(lmy) / lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA
            magz = float(lmz) / lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA
            # GYROSCOPIO
            gyrox = float(lgx) * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS
            gyroy = float(lgy) * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS
            gyroz = float(lgz) * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS            
            strOut="%ld;%f;%f;%f;%f;%f;%f;%f;%f;%f" % (long_millis,accx,accy,accz,magx,magy,magz,gyrox,gyroy,gyroz)
            self.lastmillis = long_millis
            self.datalogger.info(strOut)
        else:
            pass #self.infologger.error("Py2Log::parse_data_26b_data_message recevived data with %d bytes " % len(received_data))

    def parse_data_14ba_data_message(self, received_data):
        if len(received_data) == 10: 
            long_millis = long(ord(received_data[0])<<24) + long(ord(received_data[1])<<16) + long(ord(received_data[2])<<8) + long(ord(received_data[3]))
            str_ax = received_data[5] + received_data[4]
            str_ay = received_data[7] + received_data[6]
            str_az = received_data[9] + received_data[8]
            lax = struct.unpack( "h",str_ax)[0]
            lay = struct.unpack( "h",str_ay)[0]
            laz = struct.unpack( "h",str_az)[0]
            # ACCELEROMETRO
            accx = lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD * float(lax) / float(16)
            accy = lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD * float(lay) / float(16)
            accz = lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD * float(laz) / float(16)    
            strOut="%ld;%s;%f;%f;%f" % (long_millis,'A',accx,accy,accz)
            self.lastmillis = long_millis
            self.datalogger.info(strOut)
        else:
            pass #self.infologger.error("Py2Log::parse_data_26b_data_message recevived data with %d bytes " % len(received_data))

    #parse data with 14 bytes (accelerationa and gyriscope)
    def parse_data_14bm_data_message(self, received_data):
        if len(received_data) == 10: 
            long_millis = long(ord(received_data[0])<<24) + long(ord(received_data[1])<<16) + long(ord(received_data[2])<<8) + long(ord(received_data[3]))
            str_mx = received_data[5] + received_data[4]
            str_my = received_data[7] + received_data[6]
            str_mz = received_data[9] + received_data[8]
            lmx = struct.unpack( "h",str_mx)[0]
            lmy = struct.unpack( "h",str_my)[0]
            lmz = struct.unpack( "h",str_mz)[0]
            # MAGNETOMETRO
            magx = float(lmx) / lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA
            magy = float(lmy) / lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA
            magz = float(lmz) / lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA
            strOut="%ld;%s;%f;%f;%f" % (long_millis,'M',magx,magy,magz)
            self.lastmillis = long_millis
            self.datalogger.info(strOut)
        else:
            pass #self.infologger.error("Py2Log::parse_data_26b_data_message recevived data with %d bytes " % len(received_data))



    #parse data with 14 bytes (accelerationa and gyriscope)
    def parse_data_14bg_data_message(self, received_data):
        if len(received_data) == 10: 
            long_millis = long(ord(received_data[0])<<24) + long(ord(received_data[1])<<16) + long(ord(received_data[2])<<8) + long(ord(received_data[3]))
            str_gx = received_data[5] + received_data[4]
            str_gy = received_data[7] + received_data[6]
            str_gz = received_data[9] + received_data[8]
            lgx = struct.unpack( "h",str_gx)[0]
            lgy = struct.unpack( "h",str_gy)[0]
            lgz = struct.unpack( "h",str_gz)[0]
            # GYROSCOPIO
            gyrox = float(lgx) * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS
            gyroy = float(lgy) * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS
            gyroz = float(lgz) * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS            
            strOut="%ld;%s;%f;%f;%f" % (long_millis,'G',gyrox,gyroy,gyroz)
            self.lastmillis = long_millis
            self.datalogger.info(strOut)
        else:
            pass #self.infologger.error("Py2Log::parse_data_26b_data_message recevived data with %d bytes " % len(received_data))


    #parse data with 6 bytes (only sync)
    def parse_data_6b_sync_message(self, received_data):
        if len(received_data) == 6:
            long_millis = long(ord(received_data[0])<<24) + long(ord(received_data[1])<<16) + long(ord(received_data[2])<<8) + long(ord(received_data[3]))
            str_int = received_data[5] + received_data[4]
            intdata = struct.unpack( "h",str_int)[0]
            strOut = "RET_SYNC;%s;%ld;%s;%d" % (self.indata, long_millis,self.localDateTime,intdata)
            self.istatus = intdata
            self.lastmillis = long_millis
            self.transport.write(strOut,(config.get('ssh', 'remote_ssh_host_ip'), int(config.get('udp', 'port'))))
            logger.info(strOut)
            synclogger.info(strOut)
        else:
            logger.error("NrsReaderClientProtocol::parse_data_6b_sync_message recevived data with %d bytes " % len(received_data))
    
    #parse data with 6 bytes (only acceleration)
    def parse_data_6b_ack_message(self, received_data):
        if len(received_data) == 6:
            long_millis = long(ord(received_data[0])<<24) + long(ord(received_data[1])<<16) + long(ord(received_data[2])<<8) + long(ord(received_data[3]))
            str_int = received_data[5] + received_data[4]
            intdata = struct.unpack( "h",str_int)[0]
            strOut="ACK;%ld;%d" % (long_millis,intdata)
            self.istatus = intdata
            self.lastmillis = long_millis
            self.transport.write(strOut,(config.get('ssh', 'remote_ssh_host_ip'), int(config.get('udp', 'port'))))
            logger.info(strOut)
        else:
            logger.error("NrsReaderClientProtocol::parse_data_6b_ack_message recevived data with %d bytes " % len(received_data))

    def iterateLoopNew(self):
        while self.continueLoop:
            try:
                while self.imuserial.inWaiting():
                    self.imuserial.iterateNew()
                    time.sleep(0.001)
            except (AttributeError, serial.SerialException, OSError), e:
                # this way we can kill the thread by setting the board object
                # to None, or when the serial port is closed by board.exit()
                print "NrsReaderClientProtocol::iterateLoopNew AttributeError raised"
                break
            except Exception, e:
                # catch 'error: Bad file descriptor'
                # iterate may be called while the serial port is being closed,
                # causing an "error: (9, 'Bad file descriptor')"
                print "NrsReaderClientProtocol::iterateLoopNew Exception raised"
                if getattr(e, 'errno', None) == 9:
                    break
                try:
                    if e[0] == 9:
                        break
                except (TypeError, IndexError):
                    pass
                raise

    def iterateLoop(self):
        while self.continueLoop:
            try:
                while self.imuserial.available():
                    self.imuserial.iterate()
                    time.sleep(0.001)
            except (AttributeError, serial.SerialException, OSError), e:
                # this way we can kill the thread by setting the board object
                # to None, or when the serial port is closed by board.exit()
                print "NrsReaderClientProtocol::iterateLoop AttributeError raised"
                break
            except Exception, e:
                # catch 'error: Bad file descriptor'
                # iterate may be called while the serial port is being closed,
                # causing an "error: (9, 'Bad file descriptor')"
                print "NrsReaderClientProtocol::iterateLoop Exception raised"
                if getattr(e, 'errno', None) == 9:
                    break
                try:
                    if e[0] == 9:
                        break
                except (TypeError, IndexError):
                    pass
                raise
            
class CustomTimerService(internet.TimerService):
    
    def stopService(self):
        logger.info("CustomTimerService:stopService")
        send_log, args, kwargs = self.call
        protoReader = args[0]
        copyed_size , copyed_items = send_log(protoReader,True)
        protoReader.exitBoard()
        logger.info("CustomTimerService:stopService, send_log returns %d:%d" % (copyed_size , copyed_items))
        internet.TimerService.stopService(self)
           
class SyncMulticatstWriterSlave(object):
    
    def send_log(self, protoReader,sendAll=False):
        copyed_size , copyed_items = sendDataLog(sendAll)
        protoReader.sendLog(copyed_size, copyed_items)
        return copyed_size , copyed_items
    
    def makeService(self, dtLogger):
        logger.info("starting makeService....")     
        application = service.Application('SyncMulticatstWriterSlave')
        root = service.MultiService()
        root.setServiceParent(application)
        # WRITER
        protoWriter = NrsWriterClientProtocol(controller=self, port=int(config.get('udp', 'port')) )
        protoWriter.setDataLogger(dtLogger)
        root.addService(internet.MulticastServer(int(config.get('udp', 'port')), protoWriter))
        reactor.callInThread(protoWriter.iterateLoopNew)
        logger.info("NrsWriterClientProtocol added!")
        # CUSTOM
        root.addService(CustomTimerService(float(config.get('timer', 'send_log_interval')), self.send_log, protoWriter, False))
        logger.info("CustomTimerService added!")
        root.addService(internet.TimerService(60, am_i_alive))
        logger.info("Alive TimerService added!")
        logger.info("makeService terminated")
        return application

log_folder = config.get('logging', 'log_folder')
maxBytesMainLog = int(config.get('logging', 'BytesMain'))
maxBytesDataLog = int(config.get('logging', 'BytesData'))
maxDataLog = int(config.get('logging', 'maxDataLog'))
maxBytesSyncLog = int(config.get('logging', 'BytesSync'))
### LOGGING
logging.basicConfig(level=logging.INFO)
formatter = logging.Formatter('%(asctime)s;%(message)s')            
logger = logging.getLogger("yun_serial_writer")
#logger.propagate = False
if not os.path.exists(log_folder):
    os.makedirs(log_folder)
mainfh = handlers.RotatingFileHandler(os.path.join(log_folder,'yun_serial_writer.log'),maxBytes=maxBytesMainLog, backupCount=5)
mainfh.setFormatter(formatter)
logger.addHandler(mainfh)
logger.info("MAIN APP STARTING ON %s!" %  config.get('main', 'hostname'))
### DATA
datalogger = logging.getLogger('DataClient')
datalogger.propagate = False
datafh = handlers.RotatingFileHandler(os.path.join(log_folder,'data.log'),maxBytes=maxBytesDataLog, backupCount=maxDataLog)
datafh.setFormatter(formatter)
datalogger.addHandler(datafh)
### SYNC
synclogger = logging.getLogger('SyncClient')
synclogger.propagate = False
syncfh = handlers.RotatingFileHandler(os.path.join(log_folder,'sync.log'),maxBytes=maxBytesSyncLog, backupCount=5)
syncfh.setFormatter(formatter)
synclogger.addHandler(syncfh)
synclogger.info("============== STARTING ON %s ==============" %  config.get('main', 'hostname'))
### SERVERS
smc = SyncMulticatstWriterSlave()
application = smc.makeService(datalogger)
logger.info("SyncMulticatstWriterSlave STARTED!")