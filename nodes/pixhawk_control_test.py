#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
import random


"""
This is test code for how we can controll a pixhawk:
    It has code which should send the pixhawk a local waypoint
    as well as code to send the pixhawk a velocity
    (potentially adding acceleraion in the future, but no promis
"""


class DroneControl():
    def __init__(self):
        """ initialize variables and setup ros stuff """

        #initialize node
        rospy.init_node('pixhawk_control_test')


        """Publishers"""
        #cmd ouput
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', 
                PoseStamped, queue_size=0)

        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', 
                TwistStamped, queue_size=0)

        self.last_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.rotation = random.random()-0.5

        self.vel = 1
        self.vel_dir = 1
        
    def set_position(self, x=0, y=0, z=10, rotation=0):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = rotation

        self.pos_pub.publish(msg)

    def set_velocity(self, x=0, y=0, z=0, rotation=0):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = z
        msg.twist.angular.z = rotation

        self.vel_pub.publish(msg)

    def run_position_test(self):
        now = rospy.Time.now()
        if (now - self.last_time > rospy.Duration(2)):
            self.rotation = random.random()-0.5
            self.last_time = now
        self.set_position(rotation = self.rotation)

    def run_velocity_test(self):
        now = rospy.Time.now()
        if (now - self.last_time > rospy.Duration(2)):
            if self.vel<>0:
                self.vel_dir*=-1
            self.vel += self.vel_dir
            self.last_time = now
        self.set_velocity(x = self.vel)

        
if __name__=='__main__':
    controller = DroneControl()
    r=rospy.Rate(10)
    while not rospy.is_shutdown():
        controller.run_velocity_test()
        r.sleep()
