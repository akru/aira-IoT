#!/usr/bin/env python

import sys, rospy, pexpect
from mavros_msgs.srv import *
from mavros_msgs.msg import *

MAV_GLOBAL_FRAME = 3
MAV_CMD_WAYPOINT = 16
MAV_CMD_TAKEOFF  = 22
MAV_CMD_RTL      = 20

def takeoff(lat, lon, alt):
    w = Waypoint()
    w.frame = MAV_GLOBAL_FRAME 
    w.command = MAV_CMD_TAKEOFF
    w.is_current = True
    w.autocontinue = True
    w.x_lat = lat
    w.y_long = lon
    w.z_alt = alt
    return w

def waypoint(lat, lon, alt, delay):
    w = Waypoint()
    w.frame = MAV_GLOBAL_FRAME 
    w.command = MAV_CMD_WAYPOINT
    w.is_current = False
    w.autocontinue = True
    w.param1 = delay # Hold time in mession
    w.param2 = 2     # Position trashold in meters
    w.x_lat = lat
    w.y_long = lon
    w.z_alt = alt
    return w

def mission_planner(lat0, lon0, lat, lon):
    rtl = Waypoint()
    rtl.command = MAV_CMD_RTL
    return [takeoff(lat0, lon0, 20), waypoint(lat, lon, 50, 10), rtl]

def push_mission(waypoints):
    rospy.wait_for_service('mavros/mission/push')
    try:
        service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        service(waypoints)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_mode(mode):
    rospy.wait_for_service('mavros/set_mode')
    try:
        service = rospy.ServiceProxy('mavros/set_mode', SetMode)
        service(0, mode)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def arming():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        service(True)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def eth_coords():
    geth = pexpect.spawn('./geth_node.sh')
    geth.sendline("loadScript('gps_listener.js')")
    geth.expect('true');
    print "GPS Listener script loaded"
    while True:
        try:
            geth.expect('GPS', timeout=10000)
        except KeyboardInterrupt, e:
            print 'Terminate'
            geth.terminate()
            sys.exit(0)
        pos = geth.readline().split(' ')
        yield (float(pos[1]), float(pos[2][:-2]))

def main():
    ''' The main routine '''
    rospy.init_node('simple_mission')
    # For every coords received by `geth`
    for lat, lon in eth_coords():
        print 'Received GPS target:', lat, ',', lon
        # Write a simple mission
        waypoints = mission_planner(-35.363348, 149.165161, lat, lon)
        push_mission(waypoints)
        print 'Mission created'
        # Set manual mode
        set_mode('ACRO')
        # Enable motors 
        arming()
        # Set autopilot mode
        set_mode('AUTO')
        print 'Flyght!'

if __name__ == '__main__':
    main()
