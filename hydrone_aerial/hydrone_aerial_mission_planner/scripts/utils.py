import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *
from trajectory_msgs.msg import *
import math
from defines import *

def call_set_mode(mode, mode_ID):
    try:
        service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.wait_for_service("/mavros/set_mode")

        print(service(mode_ID, mode))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e   

def call_takeoff(altitude):
    try:
        service = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        rospy.wait_for_service("/mavros/cmd/takeoff")

        print(service(0.0, 0.0, 0.0, 0.0, altitude))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

def set_target_position(x,y,z):   
    if abs(x) < 0.05: x = 0.05
    if abs(y) < 0.05: y = 0.05
    if abs(z) < 0.05: z = 0.05

    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    if SIMULATION:
        while True:                      
            value = rospy.wait_for_message("/hydrone/odometry_sensor1/pose", Pose)
            # Closed Loop            
            if ( (value.position.x/x > 1.0-ARR_THRES) and (value.position.x/x < 1.0+ARR_THRES) and \
                 (value.position.y/y > 1.0-ARR_THRES) and (value.position.y/y < 1.0+ARR_THRES) and \
                 (value.position.z/z > 1.0-ARR_THRES) and (value.position.z/z < 1.0+ARR_THRES)):
                # print("Arrived to target point")               
                
                break                
            else:
                set_point_sim_pub.publish(pose)

            # print((value.position.z+offset_altitude)/z)
    else:
        # CLOSE THE LOOP FOR THE REAL AUV
        set_point_pub.publish(pose)

def call_land(target_x, target_y, target_z):
    if not SIMULATION:
        try:
            service = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
            rospy.wait_for_service("/mavros/cmd/land")

            print(service(0.0, 0.0, 0.0, 0.0, target_z))
            
        except rospy.ServiceException, e:
            print 'Service call failed: %s' % e
    else:
        set_target_position(target_x, target_y, target_z)
        print("Landind at the Base")

    time.sleep(2.5)

    LANDED_BASES.append([target_x, target_y])

def goto_to_target_position(incoming_x, incoming_y, incoming_z, target_x, target_y, target_z, NUMBER_STEPS_TO_TARGET, landed_offset):
    if not SIMULATION:
        call_arming(True)
        print("Armed")

    if SIMULATION:
        set_target_position(incoming_x, incoming_y, target_z)
        # print("Took Off")
    else:
        call_takeoff(target_z-landed_offset)
        print("Took off") 

    time.sleep(2.5)

    print("Target Set: "+str(target_x) + " " + str(target_y) + " " + str(target_z))

    delta_x = target_x - incoming_x
    delta_y = target_y - incoming_y
    for i in range(0, NUMBER_STEPS_TO_TARGET):
        target_x_delta = incoming_x + (delta_x*(i+1))/NUMBER_STEPS_TO_TARGET
        target_y_delta = incoming_y + (delta_y*(i+1))/NUMBER_STEPS_TO_TARGET

        if SIMULATION:
            set_target_position(target_x_delta, target_y_delta, target_z)
        else:
            set_target_position(target_x_delta, target_y_delta, target_z-landed_offset)
        # print("Target Set: "+str(target_x_delta) + " " + str(target_y_delta) + " " + str(FLYING_ALTITUTE))
       

def setup_vehicle():
    global SIMULATION

    if not SIMULATION: call_set_mode("GUIDED", 4)
    print("Mode guided set")

    time.sleep(1)

    if not SIMULATION: rospy.set_param("/mavros/vision_pose/tf/listen", True)

    time.sleep(5)

    if not SIMULATION: call_arming(True)
    print("First Arming Check")

    time.sleep(5)