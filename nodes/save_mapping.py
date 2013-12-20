#! /usr/bin/python

import os, os.path as osp
import argparse
import rospy

if __name__=='__main__':

    rospy.init_node('cam_mapping',anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--dev_id', help="device id", default="#1")
    parser.add_argument('--cam', help="camera name", default="camera1")
    parser.add_argument('__name',default='')
    parser.add_argument('__log',default='')
    vals = parser.parse_args()

    

    dev_id = vals.dev_id
    cam = vals.cam

    map_dir = os.getenv('CAMERA_MAPPING_DIR')
    cam_file = osp.join(map_dir,'dev_id'+dev_id[-1])

    try:
        cam_id = int(cam[-1])
        assert cam_id == 1 or cam_id == 2

        cam_map = "#"+str(cam_id)
        with open(cam_file,'w') as fh: fh.write(cam_map)
    except:
        print "Could not save mapping."
        pass
        
