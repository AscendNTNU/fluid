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

log_file_path = "/home/theo/catkin_ws/src/control_pipeline/fluid/log_module_position.txt"

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

def initLog(path):
    log = open(path,'w')
    log.write("Time\tPos.x\tPos.y\tPos.z\tVel.x\tVel.y\tVel.z\n")
    log.close()

def saveLog(path,x,y,z):
    log = open(path,'a')
    log.write(f"{time.time():.3f}\t{x:.3f}\t{y:.3f}\t{z:.3f}\n")
    log.close()


def main():
    rospy.init_node("module_position_publisher")
    module_position_publisher = rospy.Publisher("/sim/module_position", PoseWithCovarianceStamped, queue_size=1)
    initLog(log_file_path)

    rate = rospy.Rate(20)

    start_time = time.time()
    center = [0.0, 0.0]
    pitch_radius = 0.13 #0.13 for 30m long boat, 1.25m high waves and 3m high module
    roll_radius = 0.37 #0.37 for 10m wide boat, 1.25m high waves and 3m high module
    #We estimate that the period of the waves is 10 sec and then, we expect the mast to do one round every 10 sec.
    omega = 2.0 * math.pi / 10.0
    z = 3.0

    while not rospy.is_shutdown():
        
        x = center[0] - pitch_radius * math.cos(time.time()*omega)
        y = center[1] - roll_radius * math.sin(time.time()*omega)
        position = [x, y, z, 0.0]
        module_position_publisher.publish(coordinatesToPoseWithCovariance(position))
        print("Publishing to /sim/module_position: ", "%.3f " % position[1], "%.3f " % position[0], position[2], position[3])
        saveLog(log_file_path,position[0],position[1],position[2])

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
