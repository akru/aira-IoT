#!/usr/bin/env python
# Path estimation ROS node
# Subscribe:
# /path_estimation/path - two points in global navigation system coords
# Publish:
# /path_estimation/cost - flight cost for received `path` in finney

from math import atan2, sin, cos, sqrt, pi
from rospy import init_node, spin, Subscriber, Publisher
from dron_ros_tutorial.msg import PathCost, PathEstimate

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

def get_cost_by(length):
    return int(round(length) * 10)

def main():
    ''' The main routine '''
    init_node('quad_flight_estimation')
    pub = Publisher('/path_estimation/cost', PathCost, queue_size=1)
    
    def cost_estimation(msg):
        # Calc path length in kilometers
        length = path_length(msg.base.latitude, msg.base.longitude,
                             msg.destination.latitude, msg.destination.longitude)
        print('Path length is {0} km'.format(length))
        # Calc cost by path length
        cost = get_cost_by(length)
        print('Cost is {0} finney'.format(cost))
        # Make message and publish
        cost_msg = PathCost()
        cost_msg.ident = msg.ident
        cost_msg.cost = cost
        pub.publish(cost_msg)
    
    Subscriber('/path_estimation/path', PathEstimate, cost_estimation)
    spin()

if __name__ == '__main__':
    main()
