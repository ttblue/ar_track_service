#! /usr/bin/python

import os, os.path as osp
import argparse
import rospy

if __name__=='__main__':

    rospy.init_node('cam_mapping',anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--type', help="rgbd or rgb", default="rgbd")
    parser.add_argument('--model', help="camera model", default="creative")
    parser.add_argument('--cam', help="camera name", default="camera1")
    parser.add_argument('__name',default='')
    parser.add_argument('__log',default='')
    vals = parser.parse_args()

    cam_type = vals.type
    cam = vals.cam

    type_dir = os.getenv('CAMERA_TYPE_DIR')
    cam_file = osp.join(type_dir,cam)

    try:
        assert cam_type in ["rgbd","rgb"]
        with open(cam_file,'w') as fh: fh.write(cam_type)
        if cam_type == "rgb":
            cam_model_file = osp.join(type_dir,cam+'_model')
            with open(cam_model_file,'w') as fh: fh.write(vals.model)
    except:
        print "Invalid cam_type."
        pass
        
