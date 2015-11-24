#!/usr/bin/env python

from mission_planner import mission_planner
from geth_event import geth_event_loop
from quad_commander import *
from rospy import init_node
from math import atan2, sin, cos, sqrt, pi

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
    # For every coords received by `geth`
    for e in geth_event_loop():
        if e.event_type() == e.GPS_EVENT:
            lat = e.gps_lat
            lon = e.gps_lon
            print('Received GPS target: {0}, {1}'.format(lat, lon))
            # Write a simple mission
            waypoints = mission_planner(-35.363348, 149.165161, lat, lon)
            push_mission(waypoints)
            print('Mission created')
            # Set manual mode
            set_mode('ACRO')
            # Enable motors 
            arming()
            # Set autopilot mode
            set_mode('AUTO')
        elif e.event_type() == e.ESTIMATE_EVENT:
            print('Received estimation request: {0}'.format(e.estimate))
            length = path_length(e.estimate['from']['lat']
                                ,e.estimate['from']['lon']
                                ,e.estimate['to']['lat']
                                ,e.estimate['to']['lon'])
            print('Path length is {0} km'.format(length)) 
            cost = int(round(length * 10))
            print('Cost is {0} Eth'.format(cost))
            e.setEstimateCost(e.estimate['id'], cost)

if __name__ == '__main__':
    main()
