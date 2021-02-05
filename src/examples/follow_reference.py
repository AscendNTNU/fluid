#!/usr/bin/env python
import rospy
from math import pi, cos, sin, tan, atan2, asin, sqrt
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, AccelWithCovarianceStamped, Polygon, Quaternion
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import sys
from pathlib import Path

# This file contains many different control opportunities.
# One can chose how to control the drone by setting control_type to the appropriate CONTROL_***, for exemple CONTROL_LQR_ATTITUDE
# If LQR is used, the gains can be chosen with the variables K_lqr_x and K_lqr_y.
# If LQR is used, one can choose to multiply the feedback LQR gain by the squared root of the error in stead of the error with USE_SQRT
#  
# The drone will try to follow the drone_distance_from_mast offset in a smooth way, using 
# setpoints from transition_state.
# drone_distance_from_mast is the offset distance between the mast and the drone to ensure that
# the drone is not completely inside the mast.
# transition_state is a setpoints that smoothly move from the different iterations of the 
# drone_distance_from_mast offset to ensure that the drone stays in a steay state.
# 
# 
# 
# Parameters:
# SAVE_Z choose to store the z axis in position velocity and acceleration into the datafiles.

# common convention for Ascend is z axis upward, y axis is pointing forward and x axis to the right


SAVE_Z = True

#type of control
CONTROL_POSITION = 2552
CONTROL_VELOCITY = 2503
CONTROL_POSITION_AND_VELOCITY = 2496
CONTROL_LQR = 2111 #this is acceleration_xy_mask. We assume that we wont use it for anything else than LQR
#Attitude typemask is not the same as the one for raw setpoints. 
CONTROL_LQR_ATTITUDE = 0 # 71 should only allow pitch roll and yaw. But doesn't work

#General parameters
SAMPLE_FREQUENCY = 30.0
takeoff_height = 1.5
control_type = CONTROL_LQR_ATTITUDE
USE_SQRT = True
#K_lqr = [3.3508, 3.0525, 2.6655] #matrix from bryon's rule with diameter as max distance
a = 0.20
b = 0.40
K_lqr_x = [a*3.3508, a*3.0525, a*2.6655]
K_lqr_y = [b*3.3508, b*3.0525, b*2.6655]

MAX_ACCEL_X = 0.15 #0.3
MAX_ACCEL_Y = 0.30 #0.6


# parameters for modul position reference
center = [0.0, 0.0]
pitch_radius = 0.13  #0.13 for 30m long boat, 1.25m high waves and 3m high module
roll_radius  = 0.37  # 0.37 for 10m wide boat, 1.25m high waves and 3m high module
#We estimate that the period of the waves is 10 sec and then, we expect the mast to do one round every 10 sec.
omega = 2.0 * pi / 10.0
z = takeoff_height

#parameters for data files
reference_pose_path = str(Path.home())+"/reference_state.txt"    #file saved in home
drone_pose_path  = str(Path.home())+"/drone_state.txt"     #file saved in home
drone_setpoints_path=str(Path.home())+"/drone_setpoints.txt"     #file saved in home


# Callback for subscriber of drone position
drone_position = Point()
drone_velocity = Point()
drone_acceleration = Point()
drone_quaternion = Quaternion()
current_state = State()

target = PositionTarget()


class TransitionStruct():
    def __init__(self, max_vel, cte_acc, pose=Point(), vel=Point(), acc=Point(), sign=0):
        self.max_vel = max_vel
        self.cte_acc = cte_acc
        self.pose = pose
        self.vel = vel
        self.acc = acc
        self.transition_finished = False


local_pose_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)    
local_attitude_publisher = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)    
def poseCallback(message):
    global drone_position
    global drone_acceleration
    drone_position = message.pose.position
    drone_quaternion = message.pose.orientation
    drone_acceleration = orientation_to_acceleration(drone_quaternion)
    
def velCallback(message):
    global drone_velocity
    drone_velocity = message.twist.linear
    #printPoint(drone_velocity,"drone velocity: ")

def AccelCallback(message): #does not work with Ardupilot
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
    module_pos.z = takeoff_height
    return module_pos

def moduleVelocity(latency=None):
    module_vel = Point()
    t = rospy.Time.now()
    module_vel.x = center[0] + pitch_radius * omega * sin((rospy.Time.now().to_time()-latency)*omega)
    module_vel.y = center[1] - roll_radius  * omega * cos((rospy.Time.now().to_time()-latency)*omega)
    module_vel.z = 0.0
    return module_vel

def moduleAcceleration(latency=None):
    module_accel = Point()
    t = rospy.Time.now()
    module_accel.x = center[0] + pitch_radius * omega * omega * cos((rospy.Time.now().to_time()-latency)*omega)
    module_accel.y = center[1] + roll_radius  * omega * omega * sin((rospy.Time.now().to_time()-latency)*omega)
    module_accel.z = 0.0
    return module_accel


def derivate(actual,last):
    vel =  Point()
    vel.x = (actual.x - last.x)*SAMPLE_FREQUENCY
    vel.y = (actual.y - last.y)*SAMPLE_FREQUENCY
    vel.z = (actual.z - last.z)*SAMPLE_FREQUENCY

    #printPoint(vel,"module velocity: ")
    return vel

def calculate_lqr_acc(target_pos, trans_offset, use_sqrt=False):
    ref_pos = addPoints(target_pos[0], trans_offset.pose)
    ref_vel = addPoints(target_pos[1], trans_offset.vel)
    ref_acc = addPoints(target_pos[2], trans_offset.acc)
    accel_target = Point()
    accel_target.z = 0.0
    if use_sqrt:
        accel_target.x = K_lqr_x[0]*signed_sqrt(ref_pos.x-drone_position.x)  + K_lqr_x[1]*signed_sqrt(ref_vel.x-drone_velocity.x) + K_lqr_x[2]*signed_sqrt(ref_acc.x-drone_acceleration.x)
        accel_target.y = K_lqr_y[0]*signed_sqrt(ref_pos.y-drone_position.y)  + K_lqr_y[1]*signed_sqrt(ref_vel.y-drone_velocity.y) + K_lqr_y[2]*signed_sqrt(ref_acc.y-drone_acceleration.y)
    else:
        accel_target.x = K_lqr_x[0]*(ref_pos.x-drone_position.x)  + K_lqr_x[1]*(ref_vel.x-drone_velocity.x) + K_lqr_x[2]*(ref_acc.x-drone_acceleration.x)
        accel_target.y = K_lqr_y[0]*(ref_pos.y-drone_position.y)  + K_lqr_y[1]*(ref_vel.y-drone_velocity.y) + K_lqr_y[2]*(ref_acc.y-drone_acceleration.y)

    return accel_target

def signed_sqrt(nb):
    if nb<0:
        return - sqrt(-nb)
    else:
        return sqrt(nb)


def update_transition_state(transition_state, drone_distance_from_mast):
    #try to make a smooth transition when the relative targeted position between the drone
    #and the mast is changed

# Analysis on the x axis
    if abs(drone_distance_from_mast.x - transition_state.pose.x) > 0.001:
    # if we are in a transition state on the x axis
        transition_state.transition_finished = False
        if transition_state.vel.x ** 2  / 2.0 / transition_state.cte_acc >= abs(drone_distance_from_mast.x - transition_state.pose.x):
        # if it is time to brake to avoid overshoot
            #set the transition acceleration (or deceleration) to the one that will lead us to the exact point we want
            transition_state.acc.x = - transition_state.vel.x ** 2  /2.0 / (drone_distance_from_mast.x - transition_state.pose.x)
        elif abs(transition_state.vel.x) > transition_state.max_vel:
        # if we have reached max transitionning speed
            #we stop accelerating and maintain speed
            transition_state.acc.x = 0.0
            #transition_state.vel = signe(transition_state.vel) * transition_state.vel #to set the speed to the exact chosen value
        else:
        #we are in the acceleration phase of the transition:
            if drone_distance_from_mast.x - transition_state.pose.x > 0.0:
                transition_state.acc.x = transition_state.cte_acc
            else:
                transition_state.acc.x = - transition_state.cte_acc
        # Whatever the state we are in, update velocity and position of the target
        transition_state.vel.x  =   transition_state.vel.x  + transition_state.acc.x / SAMPLE_FREQUENCY
        transition_state.pose.x =  transition_state.pose.x  + transition_state.vel.x / SAMPLE_FREQUENCY
    else:
        if abs(transition_state.vel.x) < 0.1:
        #setpoint reached destination on this axis
            #todo: should the acceleration be tested?            
            transition_state.pose.x = drone_distance_from_mast.x
            transition_state.vel.x = 0.0
            transition_state.acc.x = 0.0

# Analysis on the y axis, same as on the x axis
    if abs(drone_distance_from_mast.y - transition_state.pose.y) > 0.001:
        transition_state.transition_finished = False
        if transition_state.vel.y ** 2  / 2.0 / transition_state.cte_acc >= abs(drone_distance_from_mast.y - transition_state.pose.y):
            transition_state.acc.y = - transition_state.vel.y ** 2  /2.0 / (drone_distance_from_mast.y - transition_state.pose.y)
        elif abs(transition_state.vel.y) > transition_state.max_vel:
            transition_state.acc.y = 0.0
        else:
            if drone_distance_from_mast.y - transition_state.pose.y > 0.0:
                transition_state.acc.y = transition_state.cte_acc
            else:
                transition_state.acc.y = - transition_state.cte_acc
        transition_state.vel.y  =   transition_state.vel.y  + transition_state.acc.y / SAMPLE_FREQUENCY
        transition_state.pose.y =  transition_state.pose.y  + transition_state.vel.y / SAMPLE_FREQUENCY
    else:
        if abs(transition_state.vel.y) < 0.1:
            transition_state.pose.y = drone_distance_from_mast.y
            transition_state.vel.y = 0.0
            transition_state.acc.y = 0.0

# Analysis on the z axis, same as on the x axis
    if abs(drone_distance_from_mast.z - transition_state.pose.z) > 0.001:
        transition_state.transition_finished = False
        if transition_state.vel.z ** 2  / 2.0 / transition_state.cte_acc >= abs(drone_distance_from_mast.z - transition_state.pose.z):
            transition_state.acc.z = - transition_state.vel.z ** 2  /2.0 / (drone_distance_from_mast.z - transition_state.pose.z)
        elif abs(transition_state.vel.z) > transition_state.max_vel:
            transition_state.acc.z = 0.0
        else:
            if drone_distance_from_mast.z - transition_state.pose.z > 0.0:
                transition_state.acc.z = transition_state.cte_acc
            else:
                transition_state.acc.z = - transition_state.cte_acc
        transition_state.vel.z  =   transition_state.vel.z  + transition_state.acc.z / SAMPLE_FREQUENCY
        transition_state.pose.z =  transition_state.pose.z  + transition_state.vel.z / SAMPLE_FREQUENCY
    else:
        if abs(transition_state.vel.z) < 0.1:
            transition_state.pose.z = drone_distance_from_mast.z
            transition_state.vel.z = 0.0
            transition_state.acc.z = 0.0
    
    if comparPoints(drone_distance_from_mast,transition_state.pose,0.001) and not transition_state.transition_finished:
        transition_state.transition_finished = True
        print ("transition finished!") 
    return transition_state

def comparPoints(pt1, pt2, r):
    if abs(pt1.x - pt2.x) >r:
        return False
    if abs(pt1.y - pt2.y) >r:
        return False
    if abs(pt1.z - pt2.z) >r:
        return False
    return True

#def signe(nb):
#    if nb > 0:
#        return 1
#    elif nb == 0:
#        return 0
#    else:
#        return -1

def addPoints(pt1,pt2):
    ret = Point()
    ret.x = pt1.x + pt2.x
    ret.y = pt1.y + pt2.y
    ret.z = pt1.z + pt2.z
    return ret

def printPoint(vec,message=None):
    rospy.loginfo(str(message)+"x: %f,\ty:%f,\tz=%f",vec.x,vec.y,vec.z)


def initLog(file_name):
    log = open(file_name,"w")
    log.write("Time\tPos.x\tPos.y")
    if SAVE_Z:
        log.write("\tPos.z")
    log.write("\tvelocity.x\tvelocity.y")
    if SAVE_Z:
        log.write("\tvelocity.z")
    log.write("\taccel.x\taccel.y")
    if SAVE_Z:
        log.write("\taccel.z")
    log.write("\n")
    log.close()

def saveLog(file_name,position,velocity=None,accel=None):
    log = open(file_name,'a')
    log.write(f"{rospy.get_rostime().secs + rospy.get_rostime().nsecs/1000000000.0:.3f}")
    log.write(f"\t{position.x:.3f}\t{position.y:.3f}")
    if SAVE_Z:
        log.write(f"\t{position.z:.3f}")
    if velocity:
        log.write(f"\t{velocity.x:.3f}\t{velocity.y:.3f}")
        if SAVE_Z:
            log.write(f"\t{velocity.z:.3f}")
    if accel:
        log.write(f"\t{accel.x:.3f}\t{accel.y:.3f}")
        if SAVE_Z:
            log.write(f"\t{accel.z:.3f}")

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
    
    for _ in range(20): #not sure how many are really useful. Has worked with only 10
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
                last_request = rospy.Time.now()
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
    yaw = 0.0
    roll = atan2(accel.x,9.81)
    pitch = atan2(accel.y,9.81)
    return euler_to_quaternion(yaw, pitch, roll)

def orientation_to_acceleration(orientation):
    accel = Point()
    angle = quaternion_to_euler_angle(orientation)
    accel.x = tan(angle.x) *9.81
    accel.y = tan(angle.y) *9.81
    accel.z = 0 #we actually don't know ...
    return accel


def euler_to_quaternion(yaw, pitch, roll):
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

def quaternion_to_euler_angle(orientation):
    ret = Point()
    w = orientation.w
    x = orientation.x
    y = orientation.y
    z = orientation.z

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    ret.x = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    ret.y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    ret.z = atan2(t3, t4)
    return ret

def constrain_acc(acc):
    #x and y axes are independant
    acc.x = constrain(acc.x, -MAX_ACCEL_X, MAX_ACCEL_X)
    acc.y = constrain(acc.y, -MAX_ACCEL_Y, MAX_ACCEL_Y)
    return acc

def constrain(x,min,max):
    if x > max:
        return max
    if x < min:
        return min
    return x

def main():
    global drone_position 
    global drone_velocity
    rospy.init_node('test_node', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velCallback)
    
    global rate
    rate = rospy.Rate(SAMPLE_FREQUENCY)

    initLog(drone_pose_path)
    initLog(reference_pose_path)
    
    #init log for acceleration setpoints. Not the same way as the rest because much shorter
    log = open(drone_setpoints_path,"w")
    log.write("Time\taccel.x\taccel.y\n")
    log.close()


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
        if USE_SQRT:
            rospy.loginfo("Drone will be controled with LQR by ATTITUDE using SQRT error!")
        else:
            rospy.loginfo("Drone will be controled with LQR by ATTITUDE")
        rospy.loginfo("K_lqr_x = %.2f, %.2f, %.2f", K_lqr_x[0], K_lqr_x[1], K_lqr_x[2]) 
        rospy.loginfo("K_lqr_y = %.2f, %.2f, %.2f", K_lqr_y[0], K_lqr_y[1], K_lqr_y[2]) 

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
    elapsed_time = 0.0

    transition_state = TransitionStruct(0.05, 0.03) # max_vel and cte_acc #first tried with 0.05 and 0.03 as these are usual values on the folowing of the mast.
    transition_state.pose = Point(0.0, 0.0, 0.0)
    drone_distance_from_mast = Point(0.0, 0.0, 0.0) # distance offset from the mast.



    while not rospy.is_shutdown():
        #estimate module state
        last_module_pose = actual_module_pose
        last_module_vel = actual_module_vel
        actual_module_pose = modulePosition()
        #actual_module_vel = derivate(actual_module_pose,last_module_pose)
        #actual_module_accel = derivate(actual_module_vel,last_module_vel)
        actual_module_vel = moduleVelocity(1.0/float(SAMPLE_FREQUENCY))
        actual_module_accel = moduleAcceleration(2.0/float(SAMPLE_FREQUENCY))

        #allow smooth movements arround the mast in its referentiel
        module_state = [actual_module_pose, actual_module_vel, actual_module_accel]
        transition_state = update_transition_state(transition_state, drone_distance_from_mast)

        acceleration_setpoint = calculate_lqr_acc(module_state, transition_state,USE_SQRT)

        #update zaxis
        actual_module_pose.z = takeoff_height + elapsed_time/50.0
        move(addPoints(actual_module_pose,transition_state.pose),actual_module_vel,actual_module_accel,CONTROL_POSITION_AND_VELOCITY)
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
        saveLog(reference_pose_path,addPoints(actual_module_pose,transition_state.pose), \
            addPoints(actual_module_vel,transition_state.vel),addPoints(actual_module_accel,transition_state.acc))

        #save log for acceleration setpoints. Not the same way as the rest because much shorter
        log = open(drone_setpoints_path,'a')
        log.write(f"{rospy.get_rostime().secs + rospy.get_rostime().nsecs/1000000000.0:.3f}")
        log.write(f"\t{acceleration_setpoint.x:.3f}\t{acceleration_setpoint.y:.3f}\n")
        log.close()

        rate.sleep()

        count = count +1

#        printPoint(transition_state.pose,"transition_state pose ")
#        printPoint(transition_state.vel, "transition_state vel")
#        printPoint(transition_state.acc, "transition_state acc")
#        printPoint(drone_distance_from_mast,"drone distance from mast ")

        if count == 30: #30 --> 1Hz
            elapsed_time = rospy.Time.now().to_time() - start_time
            print("elapsed time: %.1f" % elapsed_time)
            count = 0
            if simulation_time !=None:
                if elapsed_time >= simulation_time:
                    print("%.1fsec elapsed, simulation is over!" % elapsed_time)
                    return
            if elapsed_time >= 13.0 and elapsed_time <14.0:
                drone_distance_from_mast = Point(0.5, 0.0, 0.0)
                print("offset distance from the mast has been updated!")

        
#        rospy.loginfo("asked to pose %f,%f",x,y)
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
