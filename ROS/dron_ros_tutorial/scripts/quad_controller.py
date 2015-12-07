#!/usr/bin/env python

from mission_planner import mission_planner
from quad_commander import *

from rospy import init_node, spin, sleep, Subscriber, Publisher
from dron_ros_tutorial.msg import SatPosition
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from threading import Thread
from Queue import Queue

def drop_all(queue):
    while not queue.empty():
        queue.get()

def take_current(queue):
    drop_all(queue)
    return queue.get(True, None)

def quad_controller(target_queue, position_queue, arming_queue, homebase_pub):
    target = target_queue.get(True, None)
    print('Received GPS target: {0}, {1}'.format(target.latitude, target.longitude))
    # Store homebase
    homebase = take_current(position_queue)
    # Write a simple mission
    waypoints = mission_planner(homebase.latitude, homebase.longitude,
                                target.latitude, target.longitude)
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
    drop_all(arming_queue)
    while arming_queue.get().data:
        pass
    print('Mission complete, set homebase {0}'.format(homebase))
    satpos = SatPosition()
    satpos.latitude = homebase.latitude
    satpos.longitude = homebase.longitude
    homebase_pub.publish(satpos)

def main():
    ''' The main routine '''
    init_node('dron_employee_ros')
    # Create queues
    target_queue = Queue()
    position_queue = Queue()
    arming_queue = Queue()
    # Create homebase publisher
    homebase_pub = Publisher('/dron_employee/homebase', SatPosition, queue_size=1)
    # Create controller thread
    controller = Thread(target=quad_controller,
                        args=(target_queue, position_queue, arming_queue, homebase_pub))
    
    # Target message handler
    def quad_target(msg):
        if not controller.isAlive():
            controller.start()
            target_queue.put(msg)
        else:
            print('Target already exist!')
    
    Subscriber('/mavros/global_position/global', NavSatFix, position_queue.put) 
    Subscriber('/dron_employee/armed', Bool, arming_queue.put)
    Subscriber('/dron_employee/target', SatPosition, quad_target)
    spin()
            
if __name__ == '__main__':
    main()
