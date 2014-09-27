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
import sys, os, glob
import serial, socket
from serial import Serial
from datetime import datetime
from twisted.application import internet, service
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import threads
import logging
from logging import handlers
import ConfigParser
import time
import threading
import struct

config = ConfigParser.ConfigParser()
config.read('udp_mc_client.conf')

DATA_SYNC = 0x71
DATA_ACK = 0x7E
DATA_10B = 0x7F
DATA_26B = 0x7D
START_MSG = 0xAA
DATA_TYPE = 0x44
SYNC_TYPE = 0x53
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
        self.lastMillis = 0
        self.messaggehandlers = {}
        self.isClosing= False;
    
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
        byte = self.read()
        if not byte:
            return 0x00, received_data
        data = ord(byte)
        messaggehandler_key = 0
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


def am_i_alive():
    logger.info("Alive")

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


class Iterator(threading.Thread):
    def __init__(self, serialComm):
        super(Iterator, self).__init__()
        self.serialComm = serialComm
    
    def run(self):
        print "Iterator:run started!"
        while 1:
            try:
                while self.serialComm.available():
                    self.serialComm.iterate()
                    time.sleep(0.0005)
            except (AttributeError, serial.SerialException, OSError), e:
                # this way we can kill the thread by setting the board object
                # to None, or when the serial port is closed by board.exit()
                print "Iterator:run AttributeError raised"
                break
            except Exception, e:
                # catch 'error: Bad file descriptor'
                # iterate may be called while the serial port is being closed,
                # causing an "error: (9, 'Bad file descriptor')"
                print "Iterator:run Exception raised"
                if getattr(e, 'errno', None) == 9:
                    break
                try:
                    if e[0] == 9:
                        break
                except (TypeError, IndexError):
                    pass
                raise

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

class Py2Log:
    
    def __init__(self, port,baudrate,name, dtlogger):
        self.imuserial = ImuSerial(port, baudrate)
        self.imuserial.flushInput()
        self.imuserial.flushOutput()
        self.imuserial.add_message_handler(DATA_26B, self.parse_data_26b_data_message)  
        self.imuserial.add_message_handler(DATA_SYNC, self.parse_data_6b_sync_message)
        self.imuserial.add_message_handler(DATA_ACK,self.parse_data_6b_ack_message)
        self.datalogger = dtlogger
        self.datalogger.info("============== STARTING ON %s ==============" %  config.get('main', 'hostname'))
        self.infologger = logging.getLogger('Py2Log')
        py2fh = handlers.RotatingFileHandler(os.path.join(config.get('logging', 'log_folder'),'py2log.log'),maxBytes=int(config.get('logging', 'BytesMain')), backupCount=5)
        py2fh.setFormatter(logging.Formatter('%(asctime)s;%(message)s'))
        self.infologger.addHandler(py2fh)
        self.infologger.info("Py2Log instance created")
        self.lastmillis = 0
        self.indata = "ND"
        self.localDateTime = "ND"
 
    #parse data with 16 bytes (accelerationa and gyriscope)
    def parse_data_26b_data_message(self, received_data):
        if len(received_data) == 22: 
            long_millis = long(received_data[0]<<24) + long(received_data[1]<<16) + long(received_data[2]<<8) + long(received_data[3])
            str_ax = chr(received_data[5]) + chr(received_data[4])
            str_ay = chr(received_data[7]) + chr(received_data[6])
            str_az = chr(received_data[9]) + chr(received_data[8])
            str_mx = chr(received_data[11]) + chr(received_data[10])
            str_my = chr(received_data[13]) + chr(received_data[12])
            str_mz = chr(received_data[15]) + chr(received_data[14]) 
            str_gx = chr(received_data[17]) + chr(received_data[16])
            str_gy = chr(received_data[19]) + chr(received_data[18])
            str_gz = chr(received_data[21]) + chr(received_data[20]) 
            """lax = (received_data[5]<<8) + received_data[4]
            lay = (received_data[7]<<8) + received_data[6]
            laz = (received_data[9]<<8) + received_data[8]"""
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

    #parse data with 6 bytes (only sync)
    def parse_data_6b_sync_message(self, received_data):
        if len(received_data) == 6:
            long_millis = long(received_data[0]<<24) + long(received_data[1]<<16) + long(received_data[2]<<8) + long(received_data[3])
            strOut = "SYNC;%s;%ld;%s" % (self.indata, long_millis,self.localDateTime)
            self.lastmillis = long_millis
            synclogger.info(strOut)
        else:
            self.infologger.error("Py2Log::parse_data_6b_sync_message recevived data with %d bytes " % len(received_data))
    
    #parse data with 6 bytes (only acceleration)
    def parse_data_6b_ack_message(self, received_data):
        if len(received_data) == 6:
            long_millis = long(received_data[0]<<24) + long(received_data[1]<<16) + long(received_data[2]<<8) + long(received_data[3])
            str_int = chr(received_data[5]) + chr(received_data[4])
            intdata = struct.unpack( "h",str_int)[0]
            strOut="ACK; millis = %ld; status = %d" % (long_millis,intdata)
            self.lastmillis = long_millis
            self.infologger.info(strOut)
        else:
            self.infologger.error("Py2Log::parse_data_6b_ack_message recevived data with %d bytes " % len(received_data))
        
    def sendData(self,strdata):
        retData = self.imuserial.send_message(strdata)
        self.infologger.info("Py2Log::sendData('%s') has sent %d bytes" % (strdata,retData))

    def iterateLoop(self):
        while 1:
            try:
                while self.imuserial.available():
                    self.imuserial.iterate()
                    time.sleep(0.0005)
            except (AttributeError, serial.SerialException, OSError), e:
                # this way we can kill the thread by setting the board object
                # to None, or when the serial port is closed by board.exit()
                print "Py2Log::iterateLoop AttributeError raised"
                break
            except Exception, e:
                # catch 'error: Bad file descriptor'
                # iterate may be called while the serial port is being closed,
                # causing an "error: (9, 'Bad file descriptor')"
                print "Py2Log::iterateLoop Exception raised"
                if getattr(e, 'errno', None) == 9:
                    break
                try:
                    if e[0] == 9:
                        break
                except (TypeError, IndexError):
                    pass
                raise


    def iterate(self):
        while self.imuserial.available():
            self.imuserial.iterate()       
            
    def startIterator(self):
        it = Iterator(self.imuserial)
        it.start()
        self.infologger.info("Py2Log::startIterator Iterator Started")
        
    def stopBoard(self):
        self.infologger.info("Py2Log::stopBoard")
        self.sendData("STOP")

    def syncBoard(self,inData, localDTime):
        self.indata = inData
        self.localDateTime = localDTime
        self.infologger.info("Py2Log::syncBoard")
        self.sendData("SYNC")   
        
    def startBoard(self):
        self.infologger.info("Py2Log::startBoard")
        self.sendData("STRT")   
    
    def pauseBoard(self):
        self.infologger.info("Py2Log::pauseBoard")
        self.sendData("PAUS")
                 
    def exitBoard(self):
        self.infologger.info("Py2Log::exitBoard")
        self.imuserial.exit()
    
    def boardStatus(self):
        self.infologger.info("Py2Log::boardStatus")
        self.sendData("STAT")
        
    def dispose(self):
        try:
            self.exitBoard()
        except AttributeError:
            print "exit() raised an AttributeError unexpectedly!"+self.toString()


class NrsSyncClientProtocol(DatagramProtocol):
    noisy = False

    def __init__(self, controller, port):
        self.port = port
        self.inData = ""
        self.hostname = socket.gethostname()
       
    def set_py2log(self, py2log):
        self.py2log = py2log
        
    def startProtocol(self):
        self.transport.joinGroup(config.get('udp', 'multicast_address'))
        
    # qui deve salvare i file e spedirli via scp al server
    def sendLog(self,copyed_size, copyed_items):
        logMsg = "LOG {0}:{1}".format(copyed_items,copyed_size)
        self.transport.write(logMsg, (config.get('udp', 'multicast_address'), self.port))
        logger.info("NrsSyncClientProtocol:sendLog SENDS %s" % logMsg)


    def datagramReceived(self, datagram, (host, port)):
        if datagram[:4] == "SYNC":
            inData = datagram[5:]
            logger.info("NrsSyncClientProtocol:datagramReceived %s from %s:%d" % (inData,host, port))
            dtnow = datetime.utcnow()
            str_sync_rec = dtnow.strftime('%Y;%m;%d;%H;%M;%S;%f')
            self.py2log.syncBoard(inData,str_sync_rec)
            sMsg = "%s;%ld" % (inData,self.py2log.lastmillis)
            #dtnow = datetime.utcnow()
            #self.str_sync_ok = dtnow.strftime('%Y;%m;%d;%H;%M;%S;%f')
            pongMsg = "OKSYNC " + sMsg
            #self.transport.write(pongMsg, (config.get('udp', 'multicast_address'), self.port))
            self.transport.write(pongMsg, (host, port))
            # ricevuto il SYNC, chiedo i miei millis e li rimando al server
        if datagram[:5] == "START":
            self.py2log.startBoard()
            logger.info("NrsSyncClientProtocol:datagramReceived %s from %s:%d" % (datagram,host, port))
            self.transport.write("START received", (host, port))
        if datagram[:4] == "STOP":
            self.py2log.stopBoard()
            logger.info("NrsSyncClientProtocol:datagramReceived %s from %s:%d" % (datagram,host, port))
            self.transport.write("STOP received", (host, port))
        if datagram[:6] == "STATUS":
            self.py2log.boardStatus()
            logger.info("NrsSyncClientProtocol:datagramReceived %s from %s:%d" % (datagram,host, port))
            self.transport.write("STATUS received", (host, port))
        if datagram[:6] == "GETLOG":
            copyed_size, copyed_items = sendDataLog(True)
            self.sendLog(copyed_size, copyed_items)
        if datagram[:5] == "EXIT":
            self.py2log.exitBoard()
            self.transport.write("Board Terminated", (host, port))

class CustomTimerService(internet.TimerService):
    
    def stopService(self):
        logger.info("CustomTimerService:stopService")
        send_log, args, kwargs = self.call
        proto = args[0]
        py2log = proto.py2log
        copyed_size , copyed_items = send_log(proto,True)
        py2log.stopBoard()
        py2log.exitBoard()
        logger.info("CustomTimerService:stopService, send_log returns %d:%d" % (copyed_size , copyed_items))
        internet.TimerService.stopService(self)
           
class SyncMulticatstSlave(object):

    def send_log(self, proto,sendAll=False):
        copyed_size , copyed_items = sendDataLog(sendAll)
        proto.sendLog(copyed_size, copyed_items)
        return copyed_size , copyed_items
    
    def makeService(self,dtLogger):
        logger.info("starting makeService....")      
        p2log = Py2Log(config.get('serial', 'port'), int(config.get('serial', 'baudrate')), config.get('serial', 'name'), dtLogger  )
        logger.info("makeService Py2Log %s created!" % config.get('serial', 'name'))        
        application = service.Application('SyncMulticatstSlave')
        root = service.MultiService()
        root.setServiceParent(application)
        proto = NrsSyncClientProtocol(controller=self, port=int(config.get('udp', 'port')) )
        proto.set_py2log(p2log)
        root.addService(internet.MulticastServer(int(config.get('udp', 'port')), proto))
        logger.info("MulticastServer added!")
        root.addService(internet.TimerService(60, am_i_alive))
        #p2log.startIterator()
        threads.deferToThread(p2log.iterateLoop)
        #root.addService(internet.TimerService(0.001, p2log.iterate))
        logger.info("Iterator Started!")
        root.addService(CustomTimerService(float(config.get('timer', 'send_log_interval')), self.send_log, proto, False))
        logger.info("CustomTimerService added!")
        logger.info("makeService terminated")
        return application

log_folder = config.get('logging', 'log_folder')
maxBytesMainLog = int(config.get('logging', 'BytesMain'))
maxBytesDataLog = int(config.get('logging', 'BytesData'))
maxDataLog = int(config.get('logging', 'maxDataLog'))
maxBytesSyncLog = int(config.get('logging', 'BytesSync'))
logging.basicConfig(level=logging.INFO)
formatter = logging.Formatter('%(asctime)s;%(message)s')            
logger = logging.getLogger("udp_mc_client")
#logger.propagate = False
if not os.path.exists(log_folder):
    os.makedirs(log_folder)
mainfh = handlers.RotatingFileHandler(os.path.join(log_folder,'main.log'),maxBytes=maxBytesMainLog, backupCount=5)
mainfh.setFormatter(formatter)
logger.addHandler(mainfh)
logger.info("MAIN APP STARTING ON %s!" %  config.get('main', 'hostname'))
datalogger = logging.getLogger('DataClient')
datalogger.propagate = False
datafh = handlers.RotatingFileHandler(os.path.join(log_folder,'data.log'),maxBytes=maxBytesDataLog, backupCount=maxDataLog)
datafh.setFormatter(formatter)
datalogger.addHandler(datafh)
synclogger = logging.getLogger('SyncClient')
synclogger.propagate = False
syncfh = handlers.RotatingFileHandler(os.path.join(log_folder,'sync.log'),maxBytes=maxBytesSyncLog, backupCount=5)
syncfh.setFormatter(formatter)
synclogger.addHandler(syncfh)
synclogger.info("============== STARTING ON %s ==============" %  config.get('main', 'hostname'))
smc = SyncMulticatstSlave()
application = smc.makeService(datalogger)
logger.info("SyncMulticatstSlave STARTED!")