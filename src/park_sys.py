#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

def run():
    rospy.init_node('park_sys', anonymous=False)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass