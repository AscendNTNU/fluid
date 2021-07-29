#!/usr/bin/env python3

##
#
# Request GPS_GLOBAL_ORIGIN and mavros/global_position/set_gp_origin 
# with the received position
#
##

from threading import local
import rospy
from pymavlink import mavutil
from mavros_msgs.msg import geographic_msgs # for geographic_msgs/GeoPointStamped
from geometry_msgs.msg import Point
import math
from sensor_msgs.msg import NavSatFix

origin = 0

def print_gps(msg):
    print("Altitude is " + str(msg.altitude))
    print("\tlattitute is " + str(msg.latitude))
    print("\tlongitude is " + str(msg.longitude))


def forward_gps_origin(msg, pub):
    gp_home = geographic_msgs.msg.GeoPointStamped()
    gp_home.header.stamp = rospy.Time.now()
    #gp_home.header.frame_id = 1
    gp_home.position.altitude = msg.altitude
    gp_home.position.latitude = msg.latitude
    gp_home.position.longitude = msg.longitude
    pub.publish(gp_home)
    print("gp_home published")

def gp_callback(gp):
    if(origin):
        local_pose = Point()
        R = 6371000 #6371000 
        # In NED frame.
        local_pose.y = R* math.radians(origin.latitude/1E7-gp.latitude)
        local_pose.x = R* math.cos(math.radians(gp.latitude))*math.radians(origin.longitude/1E7-gp.longitude)
        local_pose.z = gp.altitude
        print("gps in local is x:" + str(local_pose.x) + "\ty:" + str(local_pose.y) + "\tz:" + str(local_pose.z) + "\n")

if __name__=="__main__":
    try:
        rospy.init_node("origin_publisher")
        gp_home_pub = rospy.Publisher("/mavros/global_position/set_gp_origin", geographic_msgs.msg.GeoPointStamped, queue_size=10)
        global_pose_sub = rospy.Subscriber("/mavros/global_position/global",NavSatFix,gp_callback, queue_size=10)

        # Start a connection listening to a UDP port
        the_connection = mavutil.mavlink_connection('localhost:14550')

        the_connection.wait_heartbeat()
        print("Hearbit found")
        
        #get origin GPS coordinates
        while not rospy.is_shutdown():
            #request global origin
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 49, 0, 0, 0, 0, 0, 0, 0)
            #read global origin message
            msg = the_connection.recv_match(type='GPS_GLOBAL_ORIGIN', timeout=0.1)
            if msg:
                forward_gps_origin(msg, gp_home_pub)
                print_gps(msg)
                origin = msg
                break
        rospy.sleep(1)
        while not rospy.is_shutdown():
            rospy.sleep(1)
        

    except rospy.ROSInterruptException:
        pass