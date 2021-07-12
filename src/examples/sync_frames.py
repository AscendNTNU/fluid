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

# Global position of the origin
lat = 425633500  # Terni
lon = 126432900  # Terni
alt = 163000 

the_connection = mavutil.mavlink_connection('localhost:14550')

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def send_message(msg, mav, pub):
    """
    Send a mavlink message
    """
    msg.pack(mav)
    rosmsg = convert_to_rosmsg(msg)
    pub.publish(rosmsg)

    print("sent message %s" % msg)

def set_global_origin(mav, pub):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
    to use local position information without a GPS.
    """
    target_system = mav.srcSystem
    #target_system = 0   # 0 --> broadcast to everyone
    lattitude = lat
    longitude = lon
    altitude = alt

    msg = MAV_APM.MAVLink_set_gps_global_origin_message(
            target_system,
            lattitude, 
            longitude,
            altitude)

    send_message(msg, mav, pub)


def get_global_origin(mav, pub):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
    to use local position information without a GPS.
    """
    target_system = mav.srcSystem
    #target_system = 0   # 0 --> broadcast to everyone
    lattitude = 314
    longitude = 314
    altitude = 314

    msg = MAV_APM.MAVLink_gps_global_origin_message(lattitude, longitude, altitude)
    
    #MAV_APM.MAVLINK_MSG_IDMAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_GPS_GLOBAL_ORIGIN

    print("asked GPS origin.")

    send_message(msg, mav, pub)

def set_home_position(mav, pub):
    """
    Send a mavlink SET_HOME_POSITION message, which should allow
    us to use local position information without a GPS
    """
    target_system = mav.srcSystem
    #target_system = 0  # broadcast to everyone

    lattitude = lat
    longitude = lon
    altitude = alt
    
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = MAV_APM.MAVLink_set_home_position_message(
            target_system,
            lattitude,
            longitude,
            altitude,
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z)

    send_message(msg, mav, pub)

def print_gsp_pose():
    #listening to GPS_GLOBAL_ORIGIN
    pose = the_connection.messages['GPS_GLOBAL_ORIGIN']  # Note, you can access message fields as attributes!
    alt = pose.alt
    lat = pose.lat
    lon = pose.lon
    #timestamp=the_connection.time_since('GPS_RAW_INT')
    print("Altitude is " + str(alt))
    print("\tlattitute is " + str(lat))
    print("\tlongitude is " + str(lon))
    #print("Time is " + str(timestamp))

def print_gps(msg):
    print("Altitude is " + str(msg.altitude))
    print("\tlattitute is " + str(msg.latitude))
    print("\tlongitude is " + str(msg.longitude))


if __name__=="__main__":
    try:
        rospy.init_node("origin_publisher")
        mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)
        
        # Start a connection listening to a UDP port

        # Set up mavlink instance
        f = fifo()
        mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

        msg = the_connection.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True)
        print_gps(msg)

        the_connection.wait_heartbeat()
        print("heartbit found\n")
        # wait to initialize
        while mavlink_pub.get_num_connections() <= 0:
            pass
        rospy.sleep(60)
        #get origin GPS coordinates
        print_gsp_pose()
        print("origin found!")
        for _ in range(2):
            rospy.sleep(1)
            #print_gsp_pose()
            get_global_origin(mav, mavlink_pub)
            print_gsp_pose()
            set_home_position(mav, mavlink_pub)
            set_global_origin(mav, mavlink_pub)
    except rospy.ROSInterruptException:
        pass