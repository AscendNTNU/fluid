#!/usr/bin/env python3

##################################
# This file is quite a dirty hack
# (we blame Unreal for being hard
#  to interface with)
##################################

import os
import rospy
import time
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseWithCovariance,PoseWithCovarianceStamped, Pose, Point

def coordinatesToPoseWithCovariance(coordinates):
    x = float(coordinates[0])
    y = float(coordinates[1])
    z = float(coordinates[2])
    t = float(coordinates[3])
    msg = PoseWithCovarianceStamped()
    msg.header = Header()
    msg.header.stamp.secs = int(t)
    msg.header.stamp.nsecs = int((t%1)*1000000000.0)
    msg.pose = PoseWithCovariance()
    msg.pose.pose = Pose()
    msg.pose.pose.position = Point(x,y,z)
    return msg

def main():
    rospy.init_node("module_position_publisher")
    module_position_publisher = rospy.Publisher("/sim/module_position", PoseWithCovarianceStamped, queue_size=1)
    
    rate = rospy.Rate(20) #used to be a semi colon here :o s

    start_time = time.time()
    i = 0
    center = [0.0, 0.0]
    r = 1.4
    #angle_constant = 0.01 #not used
    #We estimate that the periode of the waves is 10 sec and then, we expect the mast to do one round every 10 sec.
    omega = 2.0 * math.pi / 10.0

    while not rospy.is_shutdown():
        
        
        x = center[0] - r * math.cos(time.time()*omega)
        y = center[1] - r * math.sin(time.time()*omega)
        position = [x, y, 3.0, 0.0]
        module_position_publisher.publish(coordinatesToPoseWithCovariance(position))
        print("Publishing to /sim/module_position: ", position)
        
        i += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
