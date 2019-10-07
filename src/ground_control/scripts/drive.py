#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64,Empty
import numpy as np
import math
import tf
import sys


def error(x_desired, y_desired, x_robot, y_robot):
    '''
    Returns the euclidian distance between the robot, and the desired position

    :param x_desired: Desired robot x postition
    :type float
    :param y_desired: Desired robot y postition
    :type float
    :param x_robot: Actual robot x postition
    :type float
    :param yrobot: Actual robot y postition
    :type float

    :returns: rho
    :rtype: float
    '''

    return math.sqrt((x_desired-x_robot)**2+(y_desired-y_robot)**2)


class vehicle:

    def __init__(self):

        self.setKvalues()
        
        # Setup point to visit
        self.x_data = [11., 11., 4., 0.]
        self.y_data = [-0.5, 5, 5., 0.]
        self.xd = self.x_data[0]
        self.yd = self.y_data[0]
        self.count = 0


        self.setup_subscriptions()
        rospy.spin()

    def setKvalues(self, k_rho=0.3, k_alpha=0.85, k_beta=-0.05):
        '''
        Sets the values k_rho, k_alpha and k_beta

        :param k_rho: Robot k_rho parameter, used for the controller to update the velocity of the robot. 
            For stability k_rho > 0
        :type float
        :param k_alpha: Robot k_alpha parameter, used for the controller to update the angulare velocity of the robot. 
            For stability k_alpha-k_rho > 0
        :type float
        :param k_beta: Robot k_beta parameter, used for the controller to update the angulare velocity of the robot. 
            For stability k_beta < 0
        :type float

        '''

        if k_rho < 0 or k_beta > 0 or k_alpha-k_rho < 0:
            rospy.logwarn("Unstable k parameters set")

        self.k_p = k_rho
        self.k_a = k_alpha
        self.k_b = k_beta

    def setup_subscriptions(self):
        '''
        Sets up the different subscriptions needed by the robot
        '''

        #Velocity publisher
        self.pub = rospy.Publisher(
            '/husky_velocity_controller/cmd_vel', Twist, queue_size=1)

        #Publish rho for plotting
        self.pub2 = rospy.Publisher('/rho', Float64, queue_size=10)

        #Publisher for when destination is reached
        self.location_reached = rospy.Publisher('/dest/reached',Empty)

        #Subscribe to robot position
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.callback, queue_size=1)
        

    def callback(self, msg):
        try:
            #Get robot position from callvack
            rx = msg.pose[2].position.x
            ry = msg.pose[2].position.y

            # Calculate error
            p = error(self.xd, self.yd, rx, ry)

            # Get yaw angle from quaternion
            q = (msg.pose[2].orientation.x, msg.pose[2].orientation.y,
                 msg.pose[2].orientation.z, msg.pose[2].orientation.w)
            euler = tf.transformations.euler_from_quaternion(q)

            theta = euler[2]  # yaw

            # Set a value
            a = -theta + math.atan2(self.yd-ry, self.xd-rx)

            # Clamp 'a' between -pi:pi
            if(abs(a) > math.pi):
                if(a > 0):
                    a = 2*math.pi-a
                else:
                    a = 2*math.pi+a

            # Set beta
            b = -theta-a

            w = self.k_a*a+self.k_b*b

            # Declare varible for velocity topic
            twist = Twist()

            # Set only x velocity, in regards to robot frame (move forward)
            twist.linear.x = self.k_p*p

            # Set angular velocity
            twist.angular.z = w

            # Publish rho so it can be read by rqt
            self.pub2.publish(p)
            self.pub.publish(twist)
            if p < 0.3:
                # Set next goal
                self.location_reached.publish()
                self.count += 1
                self.xd = self.x_data[self.count % 4]
                self.yd = self.y_data[self.count % 4]
                print("new goal : x", self.xd, "y",  self.yd)

        except:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('controllnode', anonymous=True)
        rospy.loginfo("Started")
        # rospy.get_param(rospy.get_name,'/k_rho')
        

        tmp = vehicle()

    except rospy.ROSInterruptException:
        pass