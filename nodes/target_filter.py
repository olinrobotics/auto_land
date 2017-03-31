#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg

world = "/local_origin"
airframe = "/fcu_utm"
camera = "/camera"
target_raw = "/target_raw"
target_filtered = "/target"

class tf_camera_mngr:
    def __init__(self):
        rospy.init_node('tf_target_filter')

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.listener.waitForTransform(world, target_raw, rospy.Time(), rospy.Duration(60.0))

    def update(self):
        now = rospy.Time.now()

        err = False
        try:
            (trans, rot) = self.listener.lookupTransform(world, target_raw, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            err = True
        
        if  err:
            return
        
        euler_fcu = tf.transformations.euler_from_quaternion(rot)
        
        # this is a little sketcy because it doesn't directly tie the camera to the airframe
        quat_cam = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.broadcaster.sendTransform(trans,
                     quat_cam,
                     now,
                     target_filtered,
                     world)
        

if __name__ == '__main__':
    
    cam_tf = tf_camera_mngr()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        cam_tf.update()
        rate.sleep()
