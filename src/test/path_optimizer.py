#!/usr/bin/env python

import rospy

import ascend_msgs.msg
from ascend_msgs.srv import PathOptimizerService, PathOptimizerServiceResponse, PathOptimizerServiceRequest
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from math import sqrt

def constructSpline(request):
    splines = []
    timestamps = []
    path = request.path
    path.insert(0, request.pose.pose.position)
    time = 0

    for index in range(1, len(request.path)):
        start_point = path[index - 1]
        end_point = path[index]
        dx = end_point.x - start_point.x
        dy = end_point.y - start_point.y
        dz = end_point.z - start_point.z

        time += sqrt(dx*dx + dy*dy + dz*dz) * 10.0

        splines.extend([0, 0, 0, 0, dx, start_point.x])
        splines.extend([0, 0, 0, 0, dy, start_point.y])
        splines.extend([0, 0, 0, 0, dz, start_point.z])
        timestamps.append(time)

    return PathOptimizerServiceResponse(splines, timestamps)

if __name__ == '__main__':
    rospy.init_node('path_optimizer')
    service = rospy.Service("/control/path_optimizer", PathOptimizerService, constructSpline)
    rospy.wait_for_service("/control/path_optimizer")
    pathOptimizer = rospy.ServiceProxy("/control/path_optimizer", PathOptimizerService)
    pose_stamped = PoseStamped()
    twist_stamped = TwistStamped()
    imu = Imu()
    path = []
    first = Point()
    first.x = 2.0
    path.append(first)
    second = Point()
    second.x = 0.0
    second.y = 2.0
    path.append(second)
    response = pathOptimizer(pose_stamped, twist_stamped, imu, path) 
    rospy.spin()