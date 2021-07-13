#!/usr/bin/env python3

##
#
# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages
#
##

from re import L
import rospy
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from pymavlink import mavutil
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink
from mavros_msgs.msg import geographic_msgs # for geographic_msgs/GeoPointStamped


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


if __name__=="__main__":
    try:
        rospy.init_node("origin_publisher")
        gp_home_pub = rospy.Publisher("/mavros/global_position/set_gp_origin", geographic_msgs.msg.GeoPointStamped, queue_size=10)
        
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
                exit()
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass