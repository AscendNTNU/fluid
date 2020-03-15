#!/usr/bin/env python2

import rospy
import math

import ascend_msgs.msg
from fluid.srv import *
from geometry_msgs.msg import Point

finished_state = ""


def fluidOperationCompletionCallback(request):
    print "Finished operation " + request.operation
    finished_state = request.operation

    return OperationCompletionResponse()


if __name__ == '__main__':
    try:
        rospy.init_node('fluid_client')

        fluid_operation_completion_service_server = rospy.Service(
            "fluid/operation_completion", OperationCompletion, fluidOperationCompletionCallback)

        rospy.wait_for_service('fluid/take_off')
        rospy.wait_for_service('fluid/explore')

        take_off = rospy.ServiceProxy('fluid/take_off', TakeOff)
        explore = rospy.ServiceProxy('fluid/explore', Explore)

        response = take_off(3)

        if (not response.success):
            print response.message

        while not rospy.is_shutdown():

            if finished_state == "TAKE_OFF":
                rospy.logerr(finished_state)
                point = Point()
                point.x = 10

                # Doesnt get fired somehow

                rospy.logerr("hello")
                response = explore([point])

                if (not response.success):
                    print response.message

                break

        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
