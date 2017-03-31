#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import PoseStamped

"""
This is a simple version of target tracking:
    It looks at the position of the target in local coordinates
    based on the most recent update of the tf frames and commands
    the vehicle to move towards that position at an altitude of 10m
"""

LOCAL_FRAME = '/local_origin'
TARGET_FRAME = '/target'

class TargetTracker():
    def __init__(self, target_x=0, target_y=0):
        """ initialize variables and setup ros stuff """
        # These are for the option of giving an initial estimate
        # of the target's position
        self.target_x = target_x
        self.target_y = target_y
        self.trans=None

        #initialize node
        rospy.init_node('target_tracker_simple')

        """listen to target frame tf transform"""
        self.tf_frames = tf.TransformListener()

        """Publishers"""
        # publish a position setpoint to the drone
        self.cmd_pub = rospy.Publisher('/mavros/setpoint_position/local', 
                PoseStamped, queue_size=0)

    def run(self):
        try:
            (self.trans,rot) = self.tf_frames.lookupTransform(LOCAL_FRAME, TARGET_FRAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        if self.trans:
            self.target_x = self.trans[0]
            self.target_y = self.trans[1]
        self.send_cmd()

    """ Executes the control algorithms """
    def send_cmd(self):
        output = PoseStamped()
        output.pose.position.x=self.target_x
        output.pose.position.y=self.target_y
        output.pose.position.z=10

        self.cmd_pub.publish(output)


if __name__=='__main__':
    tracker = TargetTracker()
    r=rospy.Rate(50)
    while not rospy.is_shutdown():
        tracker.run()
        r.sleep()
