#!/usr/bin/env python3
from __future__ import print_function

import rospy

import random

from std_msgs.msg import Float32


def gen_number():
    rospy.init_node('random_number_generator')

    pub = rospy.Publisher('random_number', Float32)

    rospy.loginfo("Generating random numbers")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pub.publish(Float32(random.normalvariate(5, 1)))
        rate.sleep()

if __name__ == '__main__':
    try:
        gen_number()
    except KeyboardInterrupt:
        print("done")
