#!/usr/bin/env python3

##################################
# This file is quite a dirty hack
# (we blame Unreal for being hard
#  to interface with)
##################################

import os
import rospy
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseWithCovariance,PoseWithCovarianceStamped, Pose, Point, PoseStamped

def coordinatesToPoseWithCovariance(coordinates):
    x = float(coordinates[0])
    y = float(coordinates[1])
    z = float(coordinates[2])
    msg = PoseWithCovarianceStamped()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.pose = PoseWithCovariance()
    msg.pose.pose = Pose()
    msg.pose.pose.position = Point(x,y,z)
    return msg

def coordinatesToPose(coordinates):
    x = float(coordinates[0])
    y = float(coordinates[1])
    z = float(coordinates[2])
    msg = PoseStamped()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.pose = Pose()
    msg.pose.position = Point(x,y,z)
    return msg

def main():
    rospy.init_node("module_position_publisher")
    module_position_publisher = rospy.Publisher("/model_publisher/module_position", PoseWithCovarianceStamped, queue_size=1)
    module_position_withoutcov_publisher = rospy.Publisher("/model_publisher/module_position_without_cov", PoseStamped, queue_size=1)
    #initLog(log_file_path)

    rate = rospy.Rate(20)

    center = [0.0, -10.0]
    pitch_radius = 0.13 #*100 because ardupilot #0.13 for 30m long boat, 1.25m high waves and 3m high module
    roll_radius =  0.37  #*100 because ardupilot # 0.37 for 10m wide boat, 1.25m high waves and 3m high module
    #We estimate that the period of the waves is 10 sec and then, we expect the mast to do one round every 10 sec.
    omega = 2.0 * math.pi / 10.0
    z0 = 2.5

    while not rospy.is_shutdown():
        
        x = center[0] - pitch_radius * math.cos(rospy.Time.now().to_sec()*omega)
        y = center[1] - roll_radius * math.sin(rospy.Time.now().to_sec()*omega)
        z = z0 + 0.04* math.sin(rospy.Time.now().to_sec()*omega)
        position = [x, y, z]
        module_position_publisher.publish(coordinatesToPoseWithCovariance(position))
        module_position_withoutcov_publisher.publish(coordinatesToPose(position))
        #print("Publishing to /sim/module_position: ", "%.3f " % position[0], "%.3f " % position[1], position[2], position[3])
        #saveLog(log_file_path,position[1],position[0],position[2]) #save in order y,x,z to fit with the drone frame.

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
