#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg

world = "/local_origin"
airframe = "/fcu_utm"
airframe_aligned = "/fcu_aligned"
camera = "/camera"

class tf_camera_mngr:
    def __init__(self):
        rospy.init_node('tf_camera_mngr')

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.listener.waitForTransform(world, airframe, rospy.Time(), rospy.Duration(60.0))

    def update(self):
        now = rospy.Time.now()
        #self.listener.waitForTransform(world, airframe, now, rospy.Duration(2.0))
        
        err = False
        try:
            (trans, rot) = self.listener.lookupTransform(world, airframe, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            err = True
        
        if  err:
            return
        
        euler_fcu = tf.transformations.euler_from_quaternion(rot)
        
        #transform from world to camera
        quat_cam = tf.transformations.quaternion_from_euler(math.pi, 0, euler_fcu[2])
        
        self.broadcaster.sendTransform(trans,
                     quat_cam,
                     now,
                     camera,
                     world)
        
        self.broadcaster.sendTransform(trans,
                     (0, 0, 0, 1),
                     now,
                     airframe_aligned,
                     world)
        

if __name__ == '__main__':
    
    cam_tf = tf_camera_mngr()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        cam_tf.update()
        rate.sleep()
