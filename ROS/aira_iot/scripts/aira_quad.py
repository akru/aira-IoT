#!/usr/bin/env python

from mission_planner import mission_planner
from geth_event import geth_event_loop
from quad_commander import *

from math import atan2, sin, cos, sqrt, pi
from rospy import init_node, sleep, Subscriber
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
import threading

is_armed_lock = threading.Lock() 
is_armed = False
currentPosition_lock = threading.Lock()
currentPosition = (0,0)

def quad_position(msg):
    currentPosition_lock.acquire()
    global currentPosition
    currentPosition = (msg.longitude, msg.latitude)
    currentPosition_lock.release()

def quad_state(msg):
    is_armed_lock.acquire()
    global is_armed
    is_armed = msg.armed
    is_armed_lock.release()

def path_length(lat1, lon1, lat2, lon2):
    # Going to radians
    lat1 *= pi / 180
    lat2 *= pi / 180
    lon1 *= pi / 180
    lon2 *= pi / 180
    cl1 = cos(lat1)
    cl2 = cos(lat2)
    sl1 = sin(lat1)
    sl2 = sin(lat2)
    cdelta = cos(lon2 - lon1)
    sdelta = sin(lon2 - lon1)
    y = sqrt(pow(cl2 * sdelta, 2) + pow(cl1 * sl2 - sl1 * cl2 * cdelta, 2))
    x = sl1 * sl2 + cl1 * cl2 * cdelta;
    ad = atan2(y, x);
    R = 6372.795 # Radius of Earth
    return ad * R

def main():
    ''' The main routine '''
    init_node('aira_quad')
    Subscriber('/mavros/global_position/global', NavSatFix, quad_position) 
    Subscriber('/mavros/state', State, quad_state)
    spin_thread = threading.Thread(target = rospy.spin)
    spin_thread.start()
    # For every coords received by `geth`
    for e in geth_event_loop():
        if e.event_type() == e.GPS_EVENT:
            lat = e.destination['lat']
            lon = e.destination['lon']
            print('Received GPS target: {0}, {1}'.format(lat, lon))
            # Store homebase
            currentPosition_lock.acquire()
            homebase = currentPosition
            currentPosition_lock.release()
            # Write a simple mission
            waypoints = mission_planner(homebase[1], homebase[0], lat, lon)
            push_mission(waypoints)
            print('Mission created')
            # Set manual mode
            set_mode('ACRO')
            # Enable motors 
            arming()
            # Set autopilot mode
            set_mode('AUTO')
            print('Flight!')
            # Wainting for arming
            sleep(5)
            def get_armed():
                is_armed_lock.acquire()
                a = is_armed
                is_armed_lock.release()
                return a
            while get_armed():
                sleep(0.5)
            print('Mission complete, set homebase {0}'.format(homebase))
            e.setHomebase(homebase[0], homebase[1])

        elif e.event_type() == e.ESTIMATE_EVENT:
            print('Received estimation request: {0}'.format(e.estimate))
            length = path_length(e.estimate['from']['lat']
                                ,e.estimate['from']['lon']
                                ,e.estimate['to']['lat']
                                ,e.estimate['to']['lon'])
            distance = int(round(length * 1000))
            print('Path length is {0} km'.format(length)) 
            cost = int(round(length * 10))
            print('Cost is {0} Eth'.format(cost))
            e.setEstimateCost(e.estimate['id'], cost, distance)

if __name__ == '__main__':
    main()
