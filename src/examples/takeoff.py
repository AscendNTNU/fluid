#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import sys


# Callback for subscriber of drone position
drone_position = Point()
current_state = State()
local_pose_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)    

def poseCallback(message):
    print("update position\n")
    global drone_position 
    got_drone_position = True
    drone_position = message.pose.position


def state_callback(data):
    global current_state
    current_state = data

def takeoff(height):
    rospy.Subscriber("/mavros/state", State, state_callback)
    rate = rospy.Rate(30)

    # Wait for MAVROS connection with AP
    while not rospy.is_shutdown() and current_state.connected:
        rospy.loginfo("Connected to AP!")
        rate.sleep()

    # Send a few set points initially
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.z = 0.0
    pose_stamped.header.frame_id = "map"

    for _ in range(100):
        pose_stamped.header.stamp = rospy.Time.now()

        local_pose_publisher.publish(pose_stamped)
        rate.sleep()

    # Set guided and arm
    last_request = rospy.Time.now()

    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    guided_set_mode = SetMode()
    guided_set_mode.custom_mode = "GUIDED"

    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    while not rospy.is_shutdown() and not current_state.armed:

        if current_state.mode != "GUIDED" and (rospy.Time.now() - last_request) > rospy.Duration(2.0):
            if (set_mode_client.call(0, "GUIDED")):
                rospy.loginfo("Guided enabled!")
            last_request = rospy.Time.now()
        else:
            if not current_state.armed and current_state.mode == "GUIDED" and (rospy.Time.now() - last_request) > rospy.Duration(2.0):
                response = arming_client.call(True)
                if (response.success):
                    rospy.loginfo("Vehicle armed")

        pose_stamped.header.stamp = rospy.Time.now()
        local_pose_publisher.publish(pose_stamped)

        rate.sleep()

    # Send take off command

    takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
    last_request = rospy.Time.now()
    take_off_sent = False

    while not rospy.is_shutdown() and not take_off_sent:
        if (rospy.Time.now() - last_request) > rospy.Duration(3.0):
            response = takeoff_client.call(0, 0, 0, 0, height)

            if response.success:
                rospy.loginfo("Take off sent!")
                take_off_sent = True
            else:
                rospy.loginfo("Failed to send take off")
            last_request = rospy.Time.now()

        rate.sleep()
    rospy.loginfo("Take off finished")


def move(x,y,z):
    rate = rospy.Rate(30)
    if(not rospy.is_shutdown() and current_state.connected):
        rospy.loginfo("everything is clear")
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()

        local_pose_publisher.publish(pose_stamped)
    
    rate.sleep()
    
def main():
    
    rospy.init_node('test_node', anonymous=True)
    rate = rospy.Rate(30)

    height = 1.0
    takeoff(height)
    
    #waiting for takeoff to be finished
    for _ in range (5*30):
        rate.sleep()

    if len(sys.argv)>1:
        print("there is an argument, which is")
        print(sys.argv[1])
        if sys.argv[1]=="takeoff":
            rospy.loginfo("asked to only take off. Operations finished\n")
            return
    
    #while drone_position.z < height-0.2:
    #    rospy.loginfo
    #    rospy.loginfo("waiting for drone to be in altitude : %d/%d",drone_position.z,height)
    #    rate.sleep()
    x = 0
    y = 5
    rospy.loginfo("start to move to %d,%d",x,y)
    #while 1:
    move(x,y,height)
    rate.sleep()
    rospy.loginfo("asked to pose %d,%d",x,y)
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass