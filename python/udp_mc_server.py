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

$ twistd -ny udp_mc_server.py
"""
from datetime import datetime
from uuid import uuid4

from twisted.application import internet, service
from twisted.internet.protocol import DatagramProtocol
from twisted.python import log
import ntplib
import ConfigParser

config = ConfigParser.ConfigParser()
config.read('udp_mc_server.conf')

class NrsSyncProtocol(DatagramProtocol):
    noisy = False

    def __init__(self, controller, port):
        self.port = port
        self.lastSync = ""


    def startProtocol(self):
        self.transport.joinGroup(config.get('udp', 'multicast_address'))

    # qui sul master deve leggere ntp e spedire date time fino ai millis
    def sendSync(self):
        x = ntplib.NTPClient()
        dt = datetime.utcfromtimestamp(x.request('0.openwrt.pool.ntp.org').tx_time)
        dtnow = datetime.utcnow()
        self.lastSync = dtnow.strftime('%Y;%m;%d;%H;%M;%S;%f')
        sMsg = "SYNC;%s" % self.lastSync
        self.transport.write(sMsg, (config.get('udp', 'multicast_address'), self.port))
        log.msg("SEND " + sMsg)
        
    # qui sul master deve leggere ntp e spedire date time fino ai millis
    def sendStart(self):
        sMsg = "START;"
        self.transport.write(sMsg, (config.get('udp', 'multicast_address'), self.port))
        log.msg("SEND " + sMsg)
    
    def sendAcc(self):
        sMsg = "ACC;"
        self.transport.write(sMsg, (config.get('udp', 'multicast_address'), self.port))
        log.msg("SEND " + sMsg)

    # qui sul master deve leggere ntp e spedire date time fino ai millis
    def sendPause(self):
        sMsg = "PAUSE;"
        self.transport.write(sMsg, (config.get('udp', 'multicast_address'), self.port))
        log.msg("SEND " + sMsg)

    def datagramReceived(self, datagram, (host, port)):
        if datagram[:3] == "LOG":
            uuid = datagram[4:]	    # ricevuto il LOG mi preoccupo di salvarlo
            log.msg("RECV %s from %s:%d" % (datagram,host, port))
        elif datagram[:6] == "OKSYNC":
            syncmsg = datagram[7:]
            dtnow = datetime.utcnow()	    # salvo nel DB accoppiata msg con millis e host...non e detto che ricevo quanto ho mandato in self.lastSync
            log.msg("RECV %s from %s:%d" % (syncmsg,host, port))
            log.msg("....from " + self.lastSync)
            retSync = dtnow.strftime('%Y;%m;%d;%H;%M;%S;%f')
            log.msg("....received at " + retSync)



class SyncMulticatstMaster(object):

    def sync(self, proto):
        proto.sendSync()


    def makeService(self):
        application = service.Application('SyncMulticatstMaster')

        root = service.MultiService()
        root.setServiceParent(application)

        proto = NrsSyncProtocol(controller=self, port=int(config.get('udp', 'port')))
        root.addService(internet.MulticastServer(int(config.get('udp', 'port')), proto))
        root.addService(internet.TimerService(float(config.get('timer', 'sync_interval')), self.sync, proto))

        return application


application = SyncMulticatstMaster().makeService()