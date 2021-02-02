#!/usr/bin/env python
import rospy
from math import pi, cos, sin
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import sys
from pathlib import Path


#General parameters
SAMPLE_FREQUENCY = 30.0
takeoff_height = 3

# Callback for subscriber of drone position
drone_position = Point()
drone_velocity = Point()
current_state = State()
local_pose_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)    

# parameters for modul position reference
center = [0.0, 0.0]
pitch_radius = 0.13 #*100 because ardupilot #0.13 for 30m long boat, 1.25m high waves and 3m high module
roll_radius  = 0.37  #*100 because ardupilot # 0.37 for 10m wide boat, 1.25m high waves and 3m high module
#We estimate that the period of the waves is 10 sec and then, we expect the mast to do one round every 10 sec.
omega = 2.0 * pi / 10.0
z = takeoff_height

#parameters for data files
module_pose_path = str(Path.home())+"/module_position.txt"    #file saved in home
drone_pose_path  = str(Path.home())+"/drone_position.txt"     #file saved in home


velocity_mask = 2503
position_mask = 2552
position_and_velocity_mask = 2496
target = PositionTarget()
last_module_pose   = Point()
actual_module_pose = Point()

def poseCallback(message):
    global drone_position 
    drone_position = message.pose.position

def velCallback(message):
    global drone_velocity
    drone_velocity = message.twist.linear
    #printPoint(drone_velocity,"drone velocity: ")

def moduleCallback(message):
    global module_pose
    module_pose = message.pose

def stateCallback(data):
    global current_state
    current_state = data

def modulePosition():
    module_pos = Point()
    t = rospy.Time.now()
    module_pos.x = center[0] - pitch_radius * cos(rospy.Time.now().to_time()*omega)
    module_pos.y = center[1] - roll_radius  * sin(rospy.Time.now().to_time()*omega)
    module_pos.z = z
    return module_pos

def calculateModuleVelocity(actual,last):
    vel =  Point()
    vel.x = (actual.x - last.x)*SAMPLE_FREQUENCY
    vel.y = (actual.y - last.y)*SAMPLE_FREQUENCY
    vel.z = (actual.z - last.z)*SAMPLE_FREQUENCY

    #printPoint(vel,"module velocity: ")
    return vel

def printPoint(vec,message=None):
    rospy.loginfo(message+"x: %f,\ty:%f,\tz=%f",vec.x,vec.y,vec.z)


def initLog(file_name):
    log = open(file_name,"w")
    log.write("Time\tPos.x\tPos.y\tPos.z")
    #if file_name[0]==str("d"): #we want to add more info the drone logfile #bad code quality, but that's quick coding
    log.write("\tvelocity.x\tvelocity.y\tvelocity.z")
    log.write("\n")
    log.close()

def saveLog(file_name,position,velocity=None):
    log = open(file_name,'a')
    log.write(f"{rospy.get_rostime().secs + rospy.get_rostime().nsecs/1000000000.0:.3f}\t")
    log.write(f"{position.x:.3f}\t{position.y:.3f}\t{position.z:.3f}")
    if velocity:
        log.write(f"\t{velocity.x:.3f}\t{velocity.y:.3f}\t{velocity.z:.3f}")
    log.write("\n")
    log.close()

    

def takeoff(height):
    rospy.Subscriber("/mavros/state", State, stateCallback)
    global rate

    # Wait for MAVROS connection with AP
    while not rospy.is_shutdown() and current_state.connected:
        rospy.loginfo("Connected to AP!")
        rate.sleep()

    # Send a few set points initially
    target.position.z = 0.0
    
    for _ in range(100):
        target.header.stamp = rospy.Time.now()

        local_pose_publisher.publish(target)
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

        target.header.stamp = rospy.Time.now()
        local_pose_publisher.publish(target)

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

def move_follow(pose,velocity=None,type_mask=None):
    if(not rospy.is_shutdown() and current_state.connected):
        target.position.y = pose.position.y
        target.position.x = pose.position.x + 0.7
        target.position.z = pose.position.z - 0.19
        target.yaw = -3.1415
        if velocity:
            target.velocity.x = velocity.x
            target.velocity.y = velocity.y
            target.velocity.z = velocity.z
        if type_mask:
            target.type_mask = type_mask
        target.header.stamp = rospy.Time.now()
        local_pose_publisher.publish(target)
    
def move_over(pose,velocity=None,type_mask=None):
    if(not rospy.is_shutdown() and current_state.connected):
        target.position.y = pose.position.y
        target.position.x = pose.position.x + 0.32
        target.position.z = pose.position.z - 0.19
        target.yaw = -3.1415
        if velocity:
            target.velocity.x = velocity.x
            target.velocity.y = velocity.y
            target.velocity.z = velocity.z
        if type_mask:
            target.type_mask = type_mask
        target.header.stamp = rospy.Time.now()
        local_pose_publisher.publish(target)
    
def move_dock(pose,velocity=None,type_mask=None):
    if(not rospy.is_shutdown() and current_state.connected):
        target.position.y = pose.position.y
        target.position.x = pose.position.x + 0.32
        target.position.z = pose.position.z - 0.5
        target.yaw = -3.1415
        if velocity:
            target.velocity.x = velocity.x
            target.velocity.y = velocity.y
            target.velocity.z = velocity.z
        if type_mask:
            target.type_mask = type_mask
        target.header.stamp = rospy.Time.now()
        local_pose_publisher.publish(target)

def move_extract(pose,velocity=None,type_mask=None):
    if(not rospy.is_shutdown() and current_state.connected):
        target.position.y = pose.position.y
        target.position.x = pose.position.x + 0.5
        target.position.z = pose.position.z - 0.30
        target.yaw = -3.1415
        if velocity:
            target.velocity.x = velocity.x
            target.velocity.y = velocity.y
            target.velocity.z = velocity.z
        if type_mask:
            target.type_mask = type_mask
        target.header.stamp = rospy.Time.now()
        local_pose_publisher.publish(target)
    
def main():
    global drone_position 
    global drone_velocity
    global module_pose
    rospy.init_node('test_node', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velCallback)
    rospy.Subscriber("/simulator/module/pose", PoseStamped, moduleCallback)
    global rate
    rate = rospy.Rate(SAMPLE_FREQUENCY)

    initLog(drone_pose_path)
    initLog(module_pose_path)

    target.header.frame_id = "map"
    target.header.stamp = rospy.Time.now()
    target.coordinate_frame = 1
    target.type_mask = position_mask

    takeoff(takeoff_height)
    
    #waiting for takeoff to be finished
    #ugly, need to be done properly perhaps?
    
    while drone_position.z < takeoff_height /2 :
        #print("drone altitude: ",drone_position.z)
        rate.sleep()
    rospy.loginfo("Take off finished")

    if len(sys.argv)>1:
        if sys.argv[1]=="takeoff":
            rospy.loginfo("asked to only take off. Operations finished\n")
            return
    
    #while drone_position.z < takeoff_height-0.2:
    #    rospy.loginfo
    #    rospy.loginfo("waiting for drone to be in altitude : %f/%f",drone_position.z,takeoff_height)
    #    rate.sleep()
    rospy.loginfo("start to follow the module")
    actual_module_pose = module_pose
    module_velocity = Point()

    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        last_module_pose = actual_module_pose
        actual_module_pose = module_pose
        module_velocity = calculateModuleVelocity(actual_module_pose.position,last_module_pose.position)
        
        if (rospy.get_time() - start_time <= 100.0):
            print("Following")
            move_follow(actual_module_pose,module_velocity,position_and_velocity_mask)

        # elif (rospy.get_time() - start_time <= 12.0):
        #    print("Going over")
        #    move_over(actual_module_pose,module_velocity,position_and_velocity_mask)
        
        #elif (rospy.get_time() - start_time <= 15.0):
        #    print("Docking")
        #    move_over(actual_module_pose,module_velocity,position_and_velocity_mask)
        #elif (rospy.get_time() - start_time <= 15.0):
        #    print("Extracting")
        #    move_over(actual_module_pose,module_velocity,position_and_velocity_mask)
        else:
            print("Following")
            move_follow(actual_module_pose,module_velocity,position_and_velocity_mask)

        saveLog(drone_pose_path,drone_position,drone_velocity)
        saveLog(module_pose_path,actual_module_pose.position,module_velocity)
        rate.sleep()

#        rospy.loginfo("asked to pose %f,%f",x,y)
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass