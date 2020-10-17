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
        mission_planner_mapping_task()  



def find_base(starting_x, starting_y, flying_altitude):
    rospy.loginfo("Finding a Base")

    delta_x = 0
    delta_y = 0

    success = False
 
    while True:
        try:
            value = rospy.wait_for_message("/hydrone/base_detector/px_py", Float64MultiArray, 1.0)
        except(rospy.ROSException), e:
            print e   
            rospy.loginfo("NO BASE FOUND!")  
            break                      

        if value.data[0] > PX_PY_THRESHOLD:
            delta_y += -ADJUSTED_STEP
        elif value.data[0] < -PX_PY_THRESHOLD:
            delta_y += ADJUSTED_STEP
        
        if value.data[1] > PX_PY_THRESHOLD:
            delta_x += -ADJUSTED_STEP
        elif value.data[1] < -PX_PY_THRESHOLD:
            delta_x += ADJUSTED_STEP

        if ((abs(value.data[0]) < PX_PY_THRESHOLD) and (abs(value.data[1]) < PX_PY_THRESHOLD)):
            rospy.loginfo("Over the detected base")  
            # print(value.data[0], value.data[1])
            success = True
            break
        
        rospy.loginfo("APPROACHING BASE....") 

        set_target_position(starting_x+delta_x, starting_y+delta_y, flying_altitude)

    return starting_x+delta_x, starting_y+delta_y, success         




def mission_planner_mapping_task():   
    setup_vehicle()


    goto_first_hanging_base()

    rospy.loginfo("LANDING BASE....") 
    goto_to_target_position(CURRENT_POSITION[0], CURRENT_POSITION[1], CURRENT_POSITION[2], HANGING_BASE_1_POSITION[0], HANGING_BASE_1_POSITION[1], HANGING_BASE_1_POSITION[2], 5, 0.0)

    CURRENT_POSITION[0] = HANGING_BASE_1_POSITION[0]
    CURRENT_POSITION[1] = HANGING_BASE_1_POSITION[1]
    CURRENT_POSITION[2] = HANGING_BASE_1_POSITION[2]

    goto_coastal_base()


    # for i in range(0, len(DISCOVERING_POSITIONS)):
    #     target_y = DISCOVERING_POSITIONS[i][1]
    #     target_x = DISCOVERING_POSITIONS[i][0]
    #     target_z = FLYING_ALTITUTE_DISCOVERING
    #     steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)

    #     goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps, landed_offset)

    #     if SIMULATION:
    #         actual_x, actual_y, success = find_base(target_x, target_y, target_z)
    #     else:
    #         actual_x, actual_y, success = find_base(target_x, target_y, target_z-landed_offset)

    #     if (success == True):
    #         already_landed_flag = False
    #         for landed_base in LANDED_BASES:
    #             if (abs(landed_base[0] - actual_x) < 1.0 and abs(landed_base[1] - actual_y) < 1.0):
    #                 already_landed_flag = True
    #                 rospy.loginfo("BASE ALREADY LANDED!") 
    #                 break

    #         if (already_landed_flag == False):                
    #             call_land(actual_x, actual_y, 0.2)
    #             # print("Landed bases' positions:")
    #             # print(LANDED_BASES)    
    #             rospy.loginfo("LANDING...")            
    #             landed_offset = 0.0

    #     else:
    #         actual_x = DISCOVERING_POSITIONS[i][0]
    #         actual_y = DISCOVERING_POSITIONS[i][1]
    #         actual_z = FLYING_ALTITUTE_DISCOVERING

    #     if len(LANDED_BASES) == 3:
    #         break 

    # rospy.loginfo("ALL THREE LAND BASES LANDED...GOING TO HANGING BASE 2")  
    # target_x = HANGING_BASE_2_POSITION[0]
    # target_y = HANGING_BASE_2_POSITION[1]
    # target_z = FLYING_ALTITUTE_HANGING
    # steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)

    # goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps, landed_offset)
    # ############################################
    # #IMPLEMENTA FIND BASE PRA AJUSTA CHEGADA
    # #########################################
    # call_land(target_x, target_y, HANGING_BASE_2_POSITION[2])
    # landed_offset = HANGING_BASE_2_POSITION[2]

    # actual_x = HANGING_BASE_2_POSITION[0]
    # actual_y = HANGING_BASE_2_POSITION[1]
    # actual_z = FLYING_ALTITUTE_HANGING
    # # print("Landed bases' positions:")
    # # print(LANDED_BASES)
    
    # rospy.loginfo("FOUR BASES LANDED...GOING TO HANGING BASE 1") 
    # target_x = HANGING_BASE_1_POSITION[0]
    # target_y = HANGING_BASE_1_POSITION[1]
    # target_z = FLYING_ALTITUTE_HANGING
    # steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)

    # goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps, landed_offset)
    # ############################################
    # #IMPLEMENTA FIND BASE PRA AJUSTA CHEGADA
    # #########################################
    # call_land(target_x, target_y, HANGING_BASE_1_POSITION[2])
    # landed_offset = HANGING_BASE_1_POSITION[2]

    # actual_x = HANGING_BASE_1_POSITION[0]
    # actual_y = HANGING_BASE_1_POSITION[1]
    # actual_z = FLYING_ALTITUTE_HANGING
    # # print("Landed bases' positions:")
    # # print(LANDED_BASES)    

    # rospy.loginfo("ALL BASES LANDED.. GOING BACK TO COASTAL BASE...")
    # target_x = COASTAL_BASE_POSITION[0]
    # target_y = COASTAL_BASE_POSITION[1]
    # target_z = FLYING_ALTITUTE_HANGING
    # steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    
    # goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps, landed_offset)

    # call_land(target_x, target_y, COASTAL_BASE_POSITION[2])
    # print("Landed bases' positions:")
    # print(LANDED_BASES)   

    rospy.loginfo("FINISHED!!!!")
    
if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    rospy.Subscriber("/mavros/state", State, mode_callback)

    rospy.spin()