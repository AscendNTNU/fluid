#!/usr/bin/env python

import rospy

from ascend_msgs.msg import Spline
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

        time += sqrt(dx*dx + dy*dy + dz*dz)

        dx /= time
        dy /= time
        dz /= time

        spline = Spline()

        spline.x = [0, 0, 0, 0, dx, start_point.x]
        spline.y = [0, 0, 0, 0, dy, start_point.y]
        spline.z = [0, 0, 0, 0, dz, start_point.z]
        spline.timestamp = time;

        splines.append(spline)


    print(spline)

    return PathOptimizerServiceResponse(splines)

if __name__ == '__main__':
    rospy.init_node('path_optimizer')
    service = rospy.Service("/control/path_optimizer", PathOptimizerService, constructSpline)
    rospy.spin()