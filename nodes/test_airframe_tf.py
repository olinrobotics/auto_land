#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg
import random

world = "/local_origin"
airframe = "/fcu_utm"

class tf_camera_mngr:
    def __init__(self):
        rospy.init_node('tf_test')

        #self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        #self.listener.waitForTransform(world, airframe, rospy.Time(), rospy.Duration(5.0))

    def update(self):
        now = rospy.Time.now()
        
        
        
        quat_fcu = tf.transformations.quaternion_from_euler(
                (random.random()-0.5), (random.random()-0.5), random.random())
        self.broadcaster.sendTransform((1, random.random()*.5, 2),
                     quat_fcu,
                     now,
                     airframe,
                     world)


if __name__ == '__main__':
    
    cam_tf = tf_camera_mngr()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        cam_tf.update()
        rate.sleep()
