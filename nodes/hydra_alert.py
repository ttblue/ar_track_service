#!/usr/bin/python
import os
import time
import subprocess
import rospy, roslib

roslib.load_manifest('razer_hydra')
from razer_hydra.msg import Raw

threshold = 2
devnull = open(os.devnull, 'wb')

latest_time = -1
def call_back(msg):
    global latest_time
    latest_time = msg.header.stamp.to_sec()

if __name__=="__main__":
    
    rospy.init_node('hydra_alerts')
    
    sub = rospy.Subscriber('/hydra_raw', Raw, call_back)
    
    while latest_time == -1 and not rospy.is_shutdown():
        time.sleep(0.033)
        
    if not rospy.is_shutdown():
        print "First hydra message received."

    while not rospy.is_shutdown():
        while rospy.Time.now().to_sec() - latest_time > threshold:
            print "HYDRA NOT PUBLISHING."
            retcode = subprocess.call("espeak -v en 'Hydra stopped.'", stdout=devnull, stderr=devnull, shell=True)
            if retcode != 0:
                exit()
            time.sleep(0.5)
        time.sleep(0.033)            
