#!/usr/bin/python

import argparse
import rospy, roslib
roslib.load_manifest('tf')
import tf

from hd_utils.defaults import tfm_link_rof
from hd_utils import conversions

if __name__=="__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--cam', help="camera name", default="camera3")
    parser.add_argument('__name',default='')
    parser.add_argument('__log',default='')
    vals = parser.parse_args()

    rospy.init_node('cam_link_tfm_pub', anonymous=True)
    
    tfm_pub = tf.TransformBroadcaster()
    sleeper = rospy.Rate(30)

    parent = "/%s_link"%(vals.cam)
    child = "/%s_rgb_optical_frame"%(vals.cam)
    trans, rot = conversions.hmat_to_trans_rot(tfm_link_rof)

    while True:
        tfm_pub.sendTransform(trans, rot,
                              rospy.Time.now(),
                              child, parent)
        sleeper.sleep()
