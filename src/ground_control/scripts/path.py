#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point


def genArray(a):
    x = np.zeros(100,dtype='float')
    y = np.zeros(100,dtype='float')

    for i in range(x.size-1):
        x[i+1] += i/10.
        y[i+1] = a*x[i+1]
        # y[i+1] = x[i+1]*x[i+1]
    
    return x, y


def drive():
    rospy.init_node("Pathnode")
    rospy.loginfo("Data given")

    r = rospy.Rate(4)

    pub = rospy.Publisher("path",Point,queue_size=1)

    [x, y] = genArray(2)
    
    i = 0

    while not rospy.is_shutdown():
        
        p = Point()
        p.x = x[i]
        p.y = y[i]
        pub.publish(p)
        i = (i+1) % 100
        r.sleep()


if __name__ == '__main__':
    try:
        drive()

    except rospy.ROSInterruptException:
        
        pass