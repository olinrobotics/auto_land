#!/usr/bin/env python

import rospy
import tf
#from apriltags_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist

from dynamic_reconfigure.server import Server #for PID tuning
from auto_land.cfg import PIDConfig

"""
This is the main file for target tracking:
    It looks at the tf frame transformation between a target and a
    vehicle (or camera, currently), and runs PID loops to align the
    two coordinate frames.
    The output is a geometry_msgs/Twist on the cmd_vel topic
"""

BASE_FRAME = '/fcu_utm'
TARGET_FRAME = '/target'

class TargetTracker():
    def __init__(self):
        """ initialize variables and setup ros stuff """
        # initialize error
        self.offset_x=0.0
        self.offset_y=0.0
        # initialize PID output variables
        self.control_x = 0.0
        self.control_y = 0.0
        self.trans=[0,0]

        #initialize node
        rospy.init_node('target_tracker')

        """listen to target frame tf transform"""
        self.tf_frames = tf.TransformListener()

        """Subscribers"""
        # listen to PID x control output
        rospy.Subscriber('/control_x', Float64, self.control_x_callback, queue_size=10)
        # listen to PID y control output
        rospy.Subscriber('/control_y', Float64, self.control_y_callback, queue_size=10)

        """Publishers"""
        #cmd ouput
        self.pub_cmd = rospy.Publisher('/cmd_vel',Twist, queue_size=0)
        #PID publishers
        self.pub_state_x = rospy.Publisher('/state_x', Float64, queue_size=10)
        self.pub_state_y = rospy.Publisher('/state_y', Float64, queue_size=10)
        self.pub_pid_enable = rospy.Publisher('/pid_enable', Bool, queue_size=10)
        self.pub_setpoint = rospy.Publisher('/setpoint', Float64, queue_size=10)

        self.pub_setpoint.publish(Float64(0.0))
        
        # set PID values        
        self.P, self.I, self.D = 0.18, 0.0108, 0.0022

        self.set_params_x(self.P, self.I, self.D)
        self.set_params_y(self.P, self.I, self.D)
        # reversers
        self.x_dir = -1
        self.y_dir = 1
        srv = Server(PIDConfig, self.PID_callback)
        self.pub_pid_enable.publish(Bool(True))

    def PID_callback(self, config, level):
        self.P = config.P
        self.I = config.I
        self.D = config.D
        self.set_params_x(self.P, self.I, self.D)
        self.set_params_y(self.P, self.I, self.D)
        return config

    def run(self):
        try:
            (self.trans,rot) = self.tf_frames.lookupTransform(BASE_FRAME, TARGET_FRAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.offset_x=self.trans[0]
        self.offset_y=self.trans[1]

        self.pub_state_x.publish(Float64(self.offset_x))
        self.pub_state_y.publish(Float64(self.offset_y))
        self.send_cmd()


    """ Executes the control algorithms """
    def send_cmd(self):
        output = Twist()
        output.linear.x=self.control_x
        output.linear.y=self.control_y

        self.pub_cmd.publish(output)

    """ Sets PID parameters """
    def set_params_x(self, P, I, D):
        rospy.set_param('/landing_pid_x/Kp', P)
        rospy.set_param('/landing_pid_x/Ki', I)
        rospy.set_param('/landing_pid_x/Kd', D)

    def set_params_y(self, P, I, D):
        rospy.set_param('/landing_pid_y/Kp', P)
        rospy.set_param('/landing_pid_y/Ki', I)
        rospy.set_param('/landing_pid_y/Kd', D)


    """ Checks if drone has landed """
    def finished(self):
        return False


    """ Callback function for the contorl signals """
    def control_x_callback(self, data):
        self.control_x = data.data*self.x_dir
        self.control_x = self.bound(self.control_x, -1, 1)

    def control_y_callback(self, data):
        self.control_y = data.data*self.y_dir
        self.control_y = self.bound(self.control_y, -1, 1)

    def bound(self, x, lower, upper):
        if x < lower:
            return lower
        if x > upper:
            return upper
        return x

if __name__=='__main__':
    tracker = TargetTracker()
    r=rospy.Rate(50)
    while not rospy.is_shutdown():
        tracker.run()
        r.sleep()
