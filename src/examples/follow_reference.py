#!/usr/bin/env python
import rospy
from math import pi, cos, sin, atan
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, AccelWithCovarianceStamped, Polygon, Quaternion
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import sys
from pathlib import Path


#type of control
CONTROL_POSITION = 2552
CONTROL_VELOCITY = 2503
CONTROL_POSITION_AND_VELOCITY = 2496
CONTROL_LQR = 2111 #this is acceleration_xy_mask. We assume that we wont use it for anything else than LQR
CONTROL_LQR_ATTITUDE = 0 # 71

#General parameters
SAMPLE_FREQUENCY = 30.0
takeoff_height = 1.5
control_type = CONTROL_LQR_ATTITUDE
#K_lqr = [0.8377, 3.0525, 2.6655]
K_lqr_x = 16* [1.0, 0.9, 0.0]
K_lqr_y = 128* [1.0, 0.9, 0.0]


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


# Callback for subscriber of drone position
drone_position = Point()
drone_velocity = Point()
drone_acceleration = Point()
current_state = State()
local_pose_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)    
local_attitude_publisher = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)    

target = PositionTarget()

def poseCallback(message):
    global drone_position 
    drone_position = message.pose.position

def velCallback(message):
    global drone_velocity
    drone_velocity = message.twist.linear
    #printPoint(drone_velocity,"drone velocity: ")

def AccelCallback(message):
    global drone_acceleration
    drone_acceleration = message.accel.accel
    printPoint(drone_acceleration,"drone_acc ")
    

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

def derivate(actual,last):
    vel =  Point()
    vel.x = (actual.x - last.x)*SAMPLE_FREQUENCY
    vel.y = (actual.y - last.y)*SAMPLE_FREQUENCY
    vel.z = (actual.z - last.z)*SAMPLE_FREQUENCY

    #printPoint(vel,"module velocity: ")
    return vel

def calculate_lqr_acc(module_state):
    accel_target = Point()
    accel_target.z = 0
    accel_target.x = K_lqr_x[0]*(module_state[0].x-drone_position.x)  + K_lqr_x[0]*(module_state[1].x-drone_velocity.x) + K_lqr_x[0]*(module_state[2].x-drone_acceleration.x)
    accel_target.y = K_lqr_y[1]*(module_state[0].y-drone_position.y)  + K_lqr_y[1]*(module_state[1].y-drone_velocity.y) + K_lqr_y[1]*(module_state[2].y-drone_acceleration.y)

    return accel_target


def printPoint(vec,message=None):
    rospy.loginfo(message+"x: %f,\ty:%f,\tz=%f",vec.x,vec.y,vec.z)


def initLog(file_name):
    log = open(file_name,"w")
    log.write("Time\tPos.x\tPos.y\tPos.z")
    log.write("\tvelocity.x\tvelocity.y\tvelocity.z")
    log.write("\taccel.x\taccel.y\taccel.z")
    log.write("\n")
    log.close()

def saveLog(file_name,position,velocity=None,accel=None):
    log = open(file_name,'a')
    log.write(f"{rospy.get_rostime().secs + rospy.get_rostime().nsecs/1000000000.0:.3f}")
    log.write(f"\t{position.x:.3f}\t{position.y:.3f}\t{position.z:.3f}")
    if velocity:
        log.write(f"\t{velocity.x:.3f}\t{velocity.y:.3f}\t{velocity.z:.3f}")
    if accel:
        log.write(f"\t{accel.x:.3f}\t{accel.y:.3f}\t{accel.z:.3f}")
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

def move(position, velocity=None, acceleration=None, type_mask=None):
    target.position.x = position.x
    target.position.y = position.y
    target.position.z = position.z
    if velocity:
        target.velocity.x = velocity.x
        target.velocity.y = velocity.y
        target.velocity.z = velocity.z
    if acceleration:
        target.acceleration_or_force.x = acceleration.x
        target.acceleration_or_force.y = acceleration.y
        target.acceleration_or_force.z = acceleration.z
    if type_mask:
        target.type_mask = type_mask
    target.header.stamp = rospy.Time.now()
    local_pose_publisher.publish(target)

def move_attitude(orientation, control_type):
    att_targ = AttitudeTarget()
    att_targ.header.frame_id = "map"
    att_targ.header.stamp = rospy.Time.now()
    att_targ.type_mask = control_type
    att_targ.thrust = 0.5

    att_targ.orientation = orientation
    local_attitude_publisher.publish(att_targ)

def accel_to_orientation(accel,yaw=0): #yaw, double pitch, double roll) # yaw (Z), pitch (Y), roll (X)
# We consider that the drone will always be facing the mast 
# --> accel.x = roll & accel.y = pitch
    # Abbreviations for the various angular functions
    yaw = 0
    roll = atan(accel.x/9.81)
    pitch = atan(accel.y/9.81)

    #from euler angle to quaternions:
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

def constrain_acc(acc):
    if acc.x > 0.3:
        acc.x = 0.3
    elif acc.x < -0.3:
        acc.x = -0.3
    elif acc.y > 0.6:
        acc.y = 0.6
    elif acc.y < -0.6:
        acc.y = -0.6
    return acc
    
def main():
    global drone_position 
    global drone_velocity
    rospy.init_node('test_node', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velCallback)
    rospy.Subscriber("/mavros/local_position/accel", AccelWithCovarianceStamped, AccelCallback)
    
    global rate
    rate = rospy.Rate(SAMPLE_FREQUENCY)

    initLog(drone_pose_path)
    initLog(module_pose_path)

    target.header.frame_id = "map"
    target.header.stamp = rospy.Time.now()
    target.coordinate_frame = 1
    target.type_mask = CONTROL_POSITION

    if (control_type == CONTROL_POSITION):
        rospy.loginfo("Drone will be controled in POSITION")
    if (control_type == CONTROL_VELOCITY):
        rospy.loginfo("Drone will be controled in VELOCITY")
    if (control_type == CONTROL_POSITION_AND_VELOCITY):
        rospy.loginfo("Drone will be controled in POSITION and VELOCITY")
    if (control_type == CONTROL_LQR):
        rospy.loginfo("Drone will be controled with LQR")
    if (control_type == CONTROL_LQR_ATTITUDE):
        rospy.loginfo("Drone will be controled with LQR by ATTITUDE")

    simulation_time = None
    if len(sys.argv)>1:
        simulation_time = float(sys.argv[1])
        print("The simulation time has been set to %.1f" % simulation_time)
        

    
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
    last_module_pose   = Point()
    actual_module_pose = Point()
    actual_module_vel = Point()
    last_module_vel = Point()
    actual_module_pose = modulePosition()
    module_state_P = Polygon()
    module_state = module_state_P.points
    count =0
    start_time = rospy.Time.now().to_time()
    while not rospy.is_shutdown():
        
        #estimate module state
        last_module_pose = actual_module_pose
        last_module_vel = actual_module_vel
        actual_module_pose = modulePosition()
        actual_module_vel = derivate(actual_module_pose,last_module_pose)
        actual_module_accel = derivate(actual_module_vel,last_module_vel)
        
        module_state = [actual_module_pose, actual_module_vel, actual_module_accel]
        acceleration_setpoint = calculate_lqr_acc(module_state)

        #todo: don't forget to add FEEDFORWARD TO LQR!
        if(not rospy.is_shutdown() and current_state.connected):
            if control_type !=CONTROL_LQR_ATTITUDE:
                move(actual_module_pose,actual_module_vel, acceleration_setpoint,control_type)
            else:
                acceleration_setpoint = constrain_acc(acceleration_setpoint)
                orientation = accel_to_orientation(acceleration_setpoint)
                move_attitude(orientation,control_type)
        else:
            return


        saveLog(drone_pose_path,drone_position,drone_velocity,drone_acceleration)
        saveLog(module_pose_path,actual_module_pose,actual_module_vel,actual_module_accel)
        rate.sleep()

        count = count +1
        elapsed_time = rospy.Time.now().to_time() - start_time
        if count == 15: #30 --> 1Hz
            print("elapsed time: %.1f" % elapsed_time)
            count = 0
            if simulation_time !=None:
                if elapsed_time >= simulation_time:
                    print("%.1fsec elapsed, simulation is over!" % elapsed_time)
                    return

        
#        rospy.loginfo("asked to pose %f,%f",x,y)
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
