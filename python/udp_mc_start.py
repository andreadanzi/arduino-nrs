#!/usr/bin/env python

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import stdio
from twisted.protocols import basic
import ConfigParser
import sys, getopt
import ntplib
from datetime import datetime
import signal

import logging
from logging import handlers

    
config = ConfigParser.ConfigParser()
config.read('udp_mc_server.conf')

subscribed_nodes={}
message_dict={}

logging.basicConfig(level=logging.INFO)
formatter = logging.Formatter('%(asctime)s: %(message)s')            
logger = logging.getLogger("udp_mc_start")
logger.propagate = False
mainfh = handlers.RotatingFileHandler('gateway.log',maxBytes=50000, backupCount=5)
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
        self.proto.setPrintLine(self.sendLine)
    
    def connectionMade(self):
        self.transport.write('>>> ')

    def lineReceived(self, line):
        if line.startswith('h'):
            self.help()
        elif line.startswith('start'):
            logger.info( "start request")
            self.proto.sendMsg("START;")
        elif line.startswith('stop'):
            logger.info( "stop request")
            self.proto.sendMsg("STOP;")
        elif line.startswith('sync'):
            logger.info( "sync request")
            sMsg = self.proto.sendSync()
        elif line.startswith('check'):
            logger.info( "check request")
            self.proto.sendMsg("STATUS;")
        elif line.startswith('get'):
            logger.info( "get request")
            self.proto.sendMsg("GETLOG;")
        elif line.startswith('log'):
            logger.info( "log request")
            self.sendLine('Echo: ' + line)
        elif line.startswith('nodes'):
            node_num=1
            for key in subscribed_nodes:
                self.sendLine("%02d - Node with ip %s has status %s" % (node_num, key,str(subscribed_nodes[key])))
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

    def setPrintLine(self,printline):
        self.printline = printline
        
    def datagramReceived(self, datagram, (host, port)):
        logger.info( "Datagram %s received from %s" % (repr(datagram), host))
        if datagram.startswith("RET_SYNC;"):
            # "SYNC;%s;%ld;%s;%d" % (self.indata, long_millis,self.localDateTime,intdata)
            # 'SYNC;2014;10;04;07;54;52;819183;4985455;2014;10;04;07;54;52;752975;0'
            istatus = datagram.split(';')
            subscribed_nodes[host] = (istatus[-1],istatus[8])            
            self.printline("sync %s-%s" % (host,istatus[-1]))
        if datagram.startswith("ACK;"):
            #strOut="ACK;%ld;%d" % (long_millis,intdata)
            istatus = datagram.split(';')
            subscribed_nodes[host] = (istatus[-1],istatus[1])
            self.printline("ack %s-%s" % (host,istatus[-1]))
        if datagram.startswith("RET_LOG"):
            self.printline(datagram)            
            

    # qui sul master deve leggere ntp e spedire date time fino ai millis
    def sendSync(self):
        x = ntplib.NTPClient()
        dt = datetime.utcfromtimestamp(x.request('localhost').tx_time)
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
    