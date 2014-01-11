#!/usr/bin/python
import os
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
    sleeper1 = rospy.Rate(30)
    sleeper2 = rospy.Rate(0.75)
    
    while latest_time == -1:
        sleeper1.sleep()
        
    print "First hydra message received."

    while True:
        while rospy.Time.now().to_sec() - latest_time > threshold:
            print "HYDRA NOT PUBLISHING."
            try:
                retcode = subprocess.call("espeak -v en 'Hydra stopped.'", stdout=devnull, stderr=devnull, shell=True)
                if retcode != 0 :
                    exit()
            except KeyboardInterrupt:
                print "Exiting"
                exit()
            sleeper2.sleep()
        sleeper1.sleep()
            
