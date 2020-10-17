#! /usr/bin/env python
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
from utils import *


CURRENT_POSITION = [COASTAL_BASE_POSITION[0], COASTAL_BASE_POSITION[1], COASTAL_BASE_POSITION[2]]

def goto_first_hanging_base():
    target_x = HANGING_BASE_1_POSITION[0]
    target_y = HANGING_BASE_1_POSITION[1]
    target_z = FLYING_ALTITUTE_HANGING

    steps = int( math.hypot( target_x-CURRENT_POSITION[0], target_y-CURRENT_POSITION[1] ) / MAX_STEP_SIZE )

    rospy.loginfo("GOING TO THE FIRST HANGING BASE....") 
    goto_to_target_position(CURRENT_POSITION[0], CURRENT_POSITION[1], CURRENT_POSITION[2], target_x, target_y, target_z, steps, 0.0)

    CURRENT_POSITION[0] = target_x
    CURRENT_POSITION[1] = target_y
    CURRENT_POSITION[2] = target_z

    rospy.loginfo("APPROACHING BASE....") 
    goto_to_target_position(CURRENT_POSITION[0], CURRENT_POSITION[1], CURRENT_POSITION[2], target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_1_POSITION[2], 5, 0.0)

    CURRENT_POSITION[0] = target_x
    CURRENT_POSITION[1] = target_y
    CURRENT_POSITION[2] = FLYING_ALTITUTE_INSPECTING+HANGING_BASE_1_POSITION[2]



def goto_coastal_base():
    target_x = COASTAL_BASE_POSITION[0]
    target_y = COASTAL_BASE_POSITION[1]
    target_z = FLYING_ALTITUTE_HANGING

    steps = int( math.hypot( target_x-CURRENT_POSITION[0], target_y-CURRENT_POSITION[1] ) / MAX_STEP_SIZE )

    rospy.loginfo("GOING TO THE COASTAL BASE....") 
    goto_to_target_position(CURRENT_POSITION[0], CURRENT_POSITION[1], CURRENT_POSITION[2], target_x, target_y, target_z, steps, 0.0)

    target_z = COASTAL_BASE_POSITION[2]

    CURRENT_POSITION[0] = target_x
    CURRENT_POSITION[1] = target_y
    CURRENT_POSITION[2] = target_z

    call_land(target_x, target_y, COASTAL_BASE_POSITION[2]+OFFSET_Z)



def mode_callback(data):
    if data.mode == "ACRO": 
        if not SIMULATION: call_set_mode("GUIDED", 4)   
        mission_planner_inspection_task()  

def mission_planner_inspection_task(): 
     
    activate_detection = rospy.Publisher("/gas_detector/activate", Bool, queue_size=10)
    activate_detection.publish(False)

    setup_vehicle()

    goto_first_hanging_base()

    rospy.loginfo("INSPECTING....") 
    activate_detection.publish(True)
    time.sleep(INSPECTION_WAIT_TIME)
    activate_detection.publish(False)


    goto_coastal_base()



    # rospy.loginfo("GOING OVER THE HANGING BASE 2.....") 
    # actual_x = HANGING_BASE_1_POSITION[0]
    # actual_y = HANGING_BASE_1_POSITION[1]
    # actual_z = FLYING_ALTITUTE_HANGING   
    # target_x = HANGING_BASE_2_POSITION[0]
    # target_y = HANGING_BASE_2_POSITION[1]
    # target_z = FLYING_ALTITUTE_HANGING
    # steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # # Setting the target
    # goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps, 0.0)
    # rospy.loginfo("APPROACHING BASE....")
    # goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_2_POSITION[2], 5, 0.0)
    # rospy.loginfo("INSPECTING....") 
    # activate_detection.publish(True)
    # time.sleep(INSPECTION_WAIT_TIME)
    # activate_detection.publish(False)
    # rospy.loginfo("GOING UP....") 
    # goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_2_POSITION[2], target_x, target_y, target_z, 5, 0.0)

    # rospy.loginfo("GOING OVER THE LANDING BASE....")
    # actual_x = HANGING_BASE_1_POSITION[0]
    # actual_y = HANGING_BASE_1_POSITION[1]
    # actual_z = FLYING_ALTITUTE_HANGING
    # target_x = FIXED_BASES_POSITION[2][0]
    # target_y = FIXED_BASES_POSITION[2][1]
    # target_z = FLYING_ALTITUTE_INSPECTING
    # steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # # Setting the target
    # goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps, 0.0)
    # rospy.loginfo("APPROACHING BASE....")
    # goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING, 5, 0.0)
    # rospy.loginfo("INSPECTING....")
    # activate_detection.publish(True)
    # time.sleep(INSPECTION_WAIT_TIME)
    # activate_detection.publish(False)
    # rospy.loginfo("GOING UP....")
    # goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING, target_x, target_y, target_z, 5, 0.0)

    # rospy.loginfo("GOING OVER THE THIRD BASE....")
    # actual_x = FIXED_BASES_POSITION[1][0]
    # actual_y = FIXED_BASES_POSITION[1][1]
    # actual_z = FLYING_ALTITUTE_DISCOVERING  
    # target_x = FIXED_BASES_POSITION[2][0]
    # target_y = FIXED_BASES_POSITION[2][1]
    # target_z = FLYING_ALTITUTE_DISCOVERING
    # steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # # Setting the target
    # goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps, 0.0)
    # rospy.loginfo("APPROACHING BASE....")
    # goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING, 5, 0.0)
    # rospy.loginfo("INSPECTING....")
    # activate_detection.publish(True)
    # time.sleep(INSPECTION_WAIT_TIME)
    # activate_detection.publish(False)
    # rospy.loginfo("GOING UP....")
    # goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING, target_x, target_y, target_z, 5, 0.0)

    # rospy.loginfo("GOING OVER THE HANGING BASE 2....")
    # actual_x = FIXED_BASES_POSITION[2][0]
    # actual_y = FIXED_BASES_POSITION[2][1]
    # actual_z = FLYING_ALTITUTE_DISCOVERING 
    # target_x = HANGING_BASE_2_POSITION[0]
    # target_y = HANGING_BASE_2_POSITION[1]
    # target_z = FLYING_ALTITUTE_HANGING
    # steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # # Setting the target
    # goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps, 0.0)
    # rospy.loginfo("APPROACHING BASE....")
    # goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_2_POSITION[2], 5, 0.0)
    # rospy.loginfo("INSPECTING....")
    # activate_detection.publish(True)
    # time.sleep(INSPECTION_WAIT_TIME)
    # activate_detection.publish(False)
    # rospy.loginfo("GOING UP....")
    # goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_2_POSITION[2], target_x, target_y, target_z, 5, 0.0)


    rospy.loginfo("Finished!!")

if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    rospy.Subscriber("/mavros/state", State, mode_callback)

    rospy.spin()