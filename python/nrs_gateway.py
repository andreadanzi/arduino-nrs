from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import stdio
from twisted.protocols import basic
import ConfigParser
import sys, getopt
import ntplib
from datetime import datetime
import signal
import time
import logging
from logging import handlers

    
config = ConfigParser.ConfigParser()
config.read('udp_mc_server.conf')

subscribed_nodes={}
message_dict={}

logging.basicConfig(level=logging.INFO)
formatter = logging.Formatter('%(asctime)s: %(message)s')            
logger = logging.getLogger("gateway")
logger.propagate = False
mainfh = handlers.RotatingFileHandler('gateway_nrs.log',maxBytes=50000, backupCount=5)
mainfh.setFormatter(formatter)
logger.addHandler(mainfh)
logger.info("Started!")

class GetMsg(basic.LineReceiver):
    from os import linesep as delimiter

    def help(self):
        self.sendLine( "commands (%s) are:"   % self.proto.getIP() )
        self.sendLine( "\tstart"  )
        self.sendLine( "\tstop")
        self.sendLine( "\tsync")
        self.sendLine( "\tcheck")
        self.sendLine( "\tget")
        self.sendLine( "\tlog")
        self.sendLine( "\tnodes")
        self.sendLine( "\tmsg")
        self.sendLine( "\texit")

    def __init__(self, proto):
        self.proto = proto
    
    def connectionMade(self):
        self.transport.write('>>> ')

    def lineReceived(self, line):
        if line.startswith('h'):
            self.help()
        elif line.startswith('start'):
            logger.info( "start request")
            #self.proto.sendMsg("START;")
            node_num=1
            for host in subscribed_nodes:
                self.proto.sendStart(node_num,host)
                node_num += 1
                time.sleep(1)
        elif line.startswith('stop'):
            logger.info( "stop request")
            node_num=1
            for host in subscribed_nodes:
                self.proto.sendStop(node_num,host)
                node_num += 1
                time.sleep(1)
        elif line.startswith('sync'):
            logger.info( "sync request")
            sMsg = self.proto.sendSync()
        elif line.startswith('check'):
            logger.info( "check request")
            self.proto.sendMsg("STATUS;")
        elif line.startswith('get'):
            logger.info( "get request")
            node_num=1
            for host in subscribed_nodes:
                self.proto.getLog(node_num,host)
                node_num += 1
                time.sleep(1)
        elif line.startswith('log'):
            logger.info( "log request")
            self.sendLine('Echo: ' + line)
        elif line.startswith('nodes'):
            node_num=1
            for key in subscribed_nodes:
                self.sendLine("%02d - Node with ip %s has status %s" % (node_num, key,str(subscribed_nodes[key])))
                logger.info("%02d - Node with ip %s has status %s" % (node_num, key,str(subscribed_nodes[key])))
                node_num += 1
        elif line.startswith('msg'):
            logger.info( "msg request")
            self.sendLine('Echo: ' + line)
        elif line.startswith('exit'):
            reactor.stop()
        else:
            self.help()
        self.transport.write('>>> ')

class NrsStartProtocol(DatagramProtocol):

    def startProtocol(self):
        self.transport.joinGroup(config.get('udp', 'multicast_address'))
        #self.transport.write(self.sMsg, (config.get('udp', 'multicast_address'), int(config.get('udp', 'port'))))
      
    def datagramReceived(self, datagram, (host, port)):
        if datagram.startswith("RET_SYNC;"):
            # "SYNC;%s;%ld;%s;%d" % (self.indata, long_millis,self.localDateTime,intdata)
            # 'SYNC;2014;10;04;07;54;52;819183;4985455;2014;10;04;07;54;52;752975;0'
            istatus = datagram.split(';')
            dtnow = datetime.utcnow()
            lastSync = dtnow.strftime('%H:%M:%S %d/%m/%Y')
            if host in subscribed_nodes:
                a, b, c, d = subscribed_nodes[host]
                subscribed_nodes[host] = (istatus[-1],istatus[1],c, lastSync)
            else:
                subscribed_nodes[host] = (istatus[-1],istatus[8],"ND",lastSync)
            logger.info( "NRS sync(%s) at %s millis received from %s" % (istatus[-1],istatus[8], host))
            print  "NRS sync(%s) at %s millis received from %s" % (istatus[-1],istatus[8], host)
        if datagram.startswith("ACK;"):
            #strOut="ACK;%ld;%d" % (long_millis,intdata)
            istatus = datagram.split(';')
            dtnow = datetime.utcnow()
            lastAck = dtnow.strftime('%H:%M:%S %d/%m/%Y')
            if host in subscribed_nodes:
                a, b, c, d = subscribed_nodes[host]
                subscribed_nodes[host] = (istatus[-1],istatus[1],lastAck, d)
            else:
                subscribed_nodes[host] = (istatus[-1],istatus[1],lastAck, "ND")
            logger.info( "NRS ack(%s) at %s millis received from %s" % (istatus[-1], istatus[1], host))
            print "NRS ack(%s) at %s millis received from %s" % (istatus[-1], istatus[1], host)
        if datagram.startswith("RET_LOG"):
            logger.info( "NRS %s received from %s" % (repr(datagram), host))
            print "NRS %s received from %s" % (repr(datagram), host)
         
    def sendStart(self,prog,addr):
        self.transport.write("START;", (addr, int(config.get('udp', 'port'))))
        print "START SENT FOR %d %s" % (prog,addr)
       
    def sendStop(self,prog,addr):
        self.transport.write("STOP;", (addr, int(config.get('udp', 'port'))))
        print "STOP SENT FOR %d %s" % (prog,addr)
    
    def getLog(self,prog,addr):
        self.transport.write("GETLOG;", (addr, int(config.get('udp', 'port'))))
        print "GETLOG SENT FOR %d %s" % (prog,addr)
        
    # qui sul master deve leggere ntp e spedire date time fino ai millis
    def sendSync(self):
        #x = ntplib.NTPClient()
        #dt = datetime.utcfromtimestamp(x.request('localhost').tx_time)
        dtnow = datetime.utcnow()
        self.lastSync = dtnow.strftime('%Y;%m;%d;%H;%M;%S;%f')
        sMsg = "SYNC;%s" % self.lastSync
        self.sMsg = sMsg
        self.transport.write(self.sMsg, (config.get('udp', 'multicast_address'), int(config.get('udp', 'port'))))
        return sMsg
    
    def sendMsg(self, sMsg):
        self.sMsg = sMsg
        self.transport.write(self.sMsg, (config.get('udp', 'multicast_address'), int(config.get('udp', 'port'))))
    
    def setMsg(self, sMsg):
        self.sMsg = sMsg

    def getIP(self):
        return config.get('udp', 'my_ipaddress')

def main():
    proto = NrsStartProtocol()
    reactor.listenMulticast(int(config.get('udp', 'port')), proto, listenMultiple=True)
    stdio.StandardIO(GetMsg(proto))
    reactor.run()


if __name__=='__main__':
    main()
    

logger.info("Terminated!")
    
