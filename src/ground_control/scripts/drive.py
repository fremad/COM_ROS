#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
import numpy as np
import math
import tf
import sys

def error(xd,yd,xr,yr):
    return math.sqrt((xd-xr)**2+(yd-yr)**2)

class vehicle:


    def __init__(self,x = 2,y = 3,krho = 0.1, kalpha = 0.35, kbeta = -0.5):
        rospy.loginfo("Drive called")
        self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist,queue_size=1)
        self.pub2 = rospy.Publisher('/rho',Float64,queue_size=10)
        self.xd = x
        self.yd = y
        self.k_p = krho
        self.k_a = kalpha
        self.k_b = kbeta
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback, queue_size=1)
        rospy.Subscriber("/path", Point, self.setPos, queue_size=1)
        rospy.spin()

    def setPos(self, point):
        self.xd = point.x
        self.yd = point.y

    def callback(self, msg):


        try:
            rx = msg.pose[1].position.x
            ry = msg.pose[1].position.y

            p = error(self.xd,self.yd,rx,ry)

            msg.pose[1].orientation
            
            q = (msg.pose[1].orientation.x, msg.pose[1].orientation.y,msg.pose[1].orientation.z,msg.pose[1].orientation.w)
            euler = tf.transformations.euler_from_quaternion(q)

            #Yaw component
            theta = euler[2]

            a = -theta +math.atan2(self.yd-ry,self.xd-rx)

            b = -theta-a

            w = self.k_a*a+self.k_b*b

            twist = Twist()


            twist.linear.x = self.k_p*p

            twist.angular.z = w

            self.pub2.publish(p)
            self.pub.publish(twist)
        except:
            pass
        


if __name__ == '__main__':
    try:
        rospy.init_node('controllnode', anonymous=True)
        rospy.loginfo("Started")
        tmp = vehicle()

    except rospy.ROSInterruptException:
        
        pass


    # sub = rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    # rospy.loginfo("Hello")