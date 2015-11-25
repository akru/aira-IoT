import pexpect, sys

class GEthEvent:
    GPS_EVENT      = 0
    ESTIMATE_EVENT = 1

    def __init__(self, event_type, geth):
        self._event_type = event_type
        self._geth = geth

        self._parser = {}
        self._parser[self.GPS_EVENT]      = self.gps_parser
        self._parser[self.ESTIMATE_EVENT] = self.estimate_parser

        self._parser[event_type](geth.readline())

    def event_type(self):
        return self._event_type

    def gps_parser(self, string):
        pos = string.split(' ')
        self.destination = {'lon': float(pos[1]), 'lat': float(pos[2][:-2])}

    def estimate_parser(self, string):
        est = string.split(' ')
        self.estimate = {}
        self.estimate['id']   = int(est[1])
        self.estimate['from'] = {'lon': float(est[2]), 'lat': float(est[3])}
        self.estimate['to']   = {'lon': float(est[4]), 'lat': float(est[5][:-2])}

    def setEstimateCost(self, estimate_id, cost):
        cmd = 'gps.setEstimateCost.sendTransaction({0}, {1}, {{from: eth.accounts[0], gas: 500000}})'.format(estimate_id, cost)
        print 'DEBUG :: ', cmd
        self._geth.sendline(cmd)

    def setHomebase(self, longitude, latitude):
        cmd = 'gps.homebase.sendTransaction(\'{0}\', \'{1}\', {{from: eth.accounts[0], gas: 500000}})    '.format(longitude, latitude)
        print 'DEBUG :: ', cmd
        self._geth.sendline(cmd)

def geth_event_loop():
    geth = pexpect.spawn('./geth_attach.sh')
    geth.sendline("loadScript('gps_listener.js')")
    geth.expect('true');
    print "Event listener script loaded"
    while True:
        try:
            ix = geth.expect(['GPS', 'EST'], timeout=100000)
            yield GEthEvent(ix, geth)
        except KeyboardInterrupt, e:
            print 'Terminate'
            geth.terminate()
            sys.exit(0)
