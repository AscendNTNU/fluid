#!/usr/bin/env python2

import rospy
import math

import ascend_msgs.msg
from fluid.srv import *
from geometry_msgs.msg import Point


def fluidOperationCompletionCallback(request):
    print "Finished operation " + request.operation

    return OperationCompletionResponse()


if __name__ == '__main__':
    try:
        rospy.init_node('fluid_client')

        fluid_operation_completion_service_server = rospy.Service(
            "fluid/operation_completion", OperationCompletion, fluidOperationCompletionCallback)

        rospy.wait_for_service('fluid/take_off')

        take_off = rospy.ServiceProxy('fluid/take_off', TakeOff)
        response = take_off(3)

        if (not response.success):
            print response.message

        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
