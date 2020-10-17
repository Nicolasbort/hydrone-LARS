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



PIPE_FIRST_CORNER = [5.25, 3.0, 0.8]
PIPE_LAST_CORNER = [4.25, 8.0, 0.8]


CURRENT_POSITION = [COASTAL_BASE_POSITION[0], COASTAL_BASE_POSITION[1], COASTAL_BASE_POSITION[2]]

def goto_first_corner():
    target_x = PIPE_FIRST_CORNER[0]
    target_y = PIPE_FIRST_CORNER[1]
    target_z = PIPE_FIRST_CORNER[2]

    steps = int( math.hypot( target_x-CURRENT_POSITION[0], target_y-CURRENT_POSITION[1] ) / MAX_STEP_SIZE )

    rospy.loginfo("GOING TO THE FIRST CORNER....") 
    goto_to_target_position(CURRENT_POSITION[0], CURRENT_POSITION[1], CURRENT_POSITION[2], target_x, target_y, target_z, steps, 0.0)

    CURRENT_POSITION[0] = target_x
    CURRENT_POSITION[1] = target_y
    CURRENT_POSITION[2] = target_z


def goto_last_corner():
    target_x = PIPE_LAST_CORNER[0]
    target_y = PIPE_LAST_CORNER[1]
    target_z = PIPE_LAST_CORNER[2]

    steps = int( math.hypot( target_x-CURRENT_POSITION[0], target_y-CURRENT_POSITION[1] ) / MAX_STEP_SIZE )

    rospy.loginfo("GOING TO THE LAST CORNER....") 
    goto_to_target_position(CURRENT_POSITION[0], CURRENT_POSITION[1], CURRENT_POSITION[2], target_x, target_y, target_z, steps, 0.0)

    CURRENT_POSITION[0] = target_x
    CURRENT_POSITION[1] = target_y
    CURRENT_POSITION[2] = target_z



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
        mission_planner_sensoring_task()  

def mission_planner_sensoring_task():  
    activate_sensor = rospy.Publisher("/sensor_detector/activate", Bool, queue_size=10)
    activate_sensor.publish(False)

    setup_vehicle()

    goto_first_corner()

    rospy.loginfo("SENSORING THE PIPE....")
    activate_sensor.publish(True)

    goto_last_corner()
    rospy.loginfo("SENSORING DONE... GOING BACK TO COASTAL BASE")

    activate_sensor.publish(False)
    

    goto_coastal_base()


    rospy.loginfo("FINISHED!!!!")

if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    rospy.Subscriber("/mavros/state", State, mode_callback)

    rospy.spin()