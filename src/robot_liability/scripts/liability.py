#!/usr/bin/env python

from robot_liability.msg import Bytes32
from std_msgs.msg import String
from base58 import b58decode
import rospy

if __name__ == '__main__':
    rospy.init_node('liability', anonymous=True)

    result = rospy.Publisher('result', Bytes32, queue_size=5)

    def ipfs_hash(msg):
        print(msg.data)
        m = Bytes32()
        m.data = b58decode(msg.data, None)[2:].encode('hex')
        print(m.data)
        result.publish(m)

    rospy.Subscriber('result_ipfs', String, ipfs_hash)
    rospy.spin()
