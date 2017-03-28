#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg
import random

camera = "/camera"
target = "/target_raw"

class tf_camera_mngr:
    def __init__(self):
        rospy.init_node('tf_target_test')

        self.broadcaster = tf.TransformBroadcaster()

    def update(self):
        now = rospy.Time.now()
        
        
        
        quat_fcu = tf.transformations.quaternion_from_euler(
                (random.random()-0.5)*.5, (random.random()-0.5)*.5, 0)
        self.broadcaster.sendTransform((0, 0, 2),
                     quat_fcu,
                     now,
                     target,
                     camera)


if __name__ == '__main__':
    
    cam_tf = tf_camera_mngr()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        cam_tf.update()
        rate.sleep()
