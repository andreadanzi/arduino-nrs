#!/usr/bin/env python

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
import ConfigParser
import sys, getopt
import ntplib
from datetime import datetime

config = ConfigParser.ConfigParser()
config.read('udp_mc_server.conf')



class NrsStartProtocol(DatagramProtocol):

    def startProtocol(self):
        self.transport.joinGroup(config.get('udp', 'multicast_address'))
        self.transport.write(self.sMsg, (config.get('udp', 'multicast_address'), int(config.get('udp', 'port'))))

    def datagramReceived(self, datagram, address):
        print "Datagram %s received from %s" % (repr(datagram), repr(address))
    
    # qui sul master deve leggere ntp e spedire date time fino ai millis
    def sendSync(self):
        x = ntplib.NTPClient()
        dt = datetime.utcfromtimestamp(x.request('0.openwrt.pool.ntp.org').tx_time)
        dtnow = datetime.utcnow()
        self.lastSync = dtnow.strftime('%Y;%m;%d;%H;%M;%S;%f')
        sMsg = "SYNC;%s" % self.lastSync
        self.sMsg = sMsg
        return sMsg
    
    def setMsg(self, sMsg):
        self.sMsg = sMsg

def main(argv):
    try:
      opts, args = getopt.getopt(argv,"c:")
    except getopt.GetoptError: 
      print 'udp_mc_start.py -c <command>'
      sys.exit(2)
    for opt, arg in opts:
      if opt == '-c':
        print "arg is %s" % arg
        sMsg = arg
        proto = NrsStartProtocol()
        if sMsg == 'SYNC':
            sMsg = proto.sendSync()
        proto.setMsg(sMsg)
        reactor.listenMulticast(int(config.get('udp', 'port')), proto, listenMultiple=True)
        reactor.run()


if __name__=='__main__':
    main(sys.argv[1:])
    