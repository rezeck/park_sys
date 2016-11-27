#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist

def run():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('park_sys', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass