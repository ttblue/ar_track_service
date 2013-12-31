#! /usr/bin/python

import os, os.path as osp
import argparse
import rospy

if __name__=='__main__':

    rospy.init_node('cam_mapping',anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--dev_id', help="device id", default="")
    parser.add_argument('--cam', help="camera name", default="camera1")
    parser.add_argument('--dev_video', help="/dev/video?", default="")
    parser.add_argument('__name',default='')
    parser.add_argument('__log',default='')
    vals = parser.parse_args()

    

    dev_id = vals.dev_id
    cam = vals.cam
    dev_video = vals.dev_video

    map_dir = os.getenv('CAMERA_MAPPING_DIR')


    try:
        if dev_video=="":
            cam_file = osp.join(map_dir,'dev_id'+dev_id[-1])
            cam_id = cam[-1]
            assert cam_id in ["1","2"]

            cam_map = "#"+cam_id
            with open(cam_file,'w') as fh: fh.write(cam_map)
        else:
            cam_file = osp.join(map_dir,cam)
            assert dev_video in ["0","1","2"]
            with open(cam_file,'w') as fh: fh.write(dev_video)

    except:
        print "Could not save mapping."
        pass
        
