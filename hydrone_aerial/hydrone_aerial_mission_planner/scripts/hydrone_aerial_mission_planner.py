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

SIMULATION = True
ARR_THRES = 0.1
OFFSET_Z = 0.2
MAX_STEP_SIZE = 0.2

FLYING_ALTITUTE_HANGING = 2.5
FLYING_ALTITUTE_DISCOVERING = 1.5
FLYING_ALTITUTE_SENSORING = 1.0
FLYING_ALTITUTE_INSPECTING = 0.25

DISCOVERING_POSITIONS = [   [0.0, -2.0], [2.0, -1.5], [4.0, -1.5], [5.0, -0.5], [6.0, -0.5], [6.0, -1.5],
                            [6.0, -3.5], [4.0, -3.5], [2.0, -3.5], [0.0, -3.5],
                            [0.0, -5.5], [2.0, -5.5], [4.0, -5.5], [6.0, -5.0]                             
                        ]

COASTAL_BASE_POSITION = [0.0, 0.0, 0.5]
HANGING_BASE_1_POSITION = [2.75, 0.0, 1.7]
HANGING_BASE_2_POSITION = [6.25, -6.0, 1.7]
CENTER_ARENA_POSITION = [3.25, -3.0]

INSPECTION_WAIT_TIME = 25

FIXED_BASES_POSITION = [[0.15, -5.0], [3.66, -3.9], [4.66, 0.15]]

LANDED_BASES = []

PX_PY_THRESHOLD = 20
ADJUSTED_STEP = 0.01

CAN_POSITIONS = [[0.25, -6.45], [6.15, 0.45]]

def call_set_mode(mode, mode_ID):
    try:
        service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.wait_for_service("/mavros/set_mode")

        print(service(mode_ID, mode))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e   

def mode_callback(data):
    if data.mode == "ACRO": 
        if not SIMULATION: call_set_mode("GUIDED", 4)   
        mission_planner_inspection_task()  

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
    if z <= 0.2: offset_altitude = 0.0
    else: offset_altitude = OFFSET_Z

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
                 ((value.position.z+offset_altitude)/z > 1.0-ARR_THRES) and ((value.position.z+offset_altitude)/z < 1.0+ARR_THRES)):
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

def find_base(starting_x, starting_y, flying_altitude):
    print("Finding a Base")

    delta_x = 0
    delta_y = 0

    success = False
 
    while True:
        try:
            value = rospy.wait_for_message("/hydrone/base_detector/px_py", Float64MultiArray, 1.0)
        except(rospy.ROSException), e:
            print e   
            print("NO BASE FOUND!")           
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
            print("Over the detected base")
            print(value.data[0], value.data[1])
            success = True
            break

        set_target_position(starting_x+delta_x, starting_y+delta_y, flying_altitude)

    return starting_x+delta_x, starting_y+delta_y, success         

def goto_to_target_position(incoming_x, incoming_y, incoming_z, target_x, target_y, target_z, NUMBER_STEPS_TO_TARGET):
    if not SIMULATION:
        call_arming(True)
        print("Armed")

    if SIMULATION:
        set_target_position(incoming_x, incoming_y, target_z)
        # print("Took Off")

        time.sleep(2.5)

        print("Target Set: "+str(target_x) + " " + str(target_y) + " " + str(target_z))

        delta_x = target_x - incoming_x
        delta_y = target_y - incoming_y
        for i in range(0, NUMBER_STEPS_TO_TARGET):
            target_x_delta = incoming_x + (delta_x*(i+1))/NUMBER_STEPS_TO_TARGET
            target_y_delta = incoming_y + (delta_y*(i+1))/NUMBER_STEPS_TO_TARGET

            set_target_position(target_x_delta, target_y_delta, target_z)
            # print("Target Set: "+str(target_x_delta) + " " + str(target_y_delta) + " " + str(FLYING_ALTITUTE))

            # time.sleep(0.5)   
    else:
        call_takeoff(2.0)
        print("Took off")  

        set_target_position(2.8, 0.0, 2.0)
        print("Target set")

def setup_vehicle():
    if not SIMULATION: call_set_mode("GUIDED", 4)
    print("Mode guided set")

    time.sleep(1)

    if not SIMULATION: rospy.set_param("/mavros/vision_pose/tf/listen", True)

    time.sleep(5)

    if not SIMULATION: call_arming(True)
    print("First Arming Check")

    time.sleep(5)

def mission_planner_mapping_task():    
    setup_vehicle()

    actual_x = COASTAL_BASE_POSITION[0]
    actual_y = COASTAL_BASE_POSITION[1]
    actual_z = COASTAL_BASE_POSITION[2]

    for i in range(0, len(DISCOVERING_POSITIONS)):
        target_y = DISCOVERING_POSITIONS[i][1]
        target_x = DISCOVERING_POSITIONS[i][0]
        target_z = FLYING_ALTITUTE_DISCOVERING
        steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)

        goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)

        actual_x, actual_y, success = find_base(target_x, target_y, target_z)

        if (success == True):
            already_landed_flag = False
            for landed_base in LANDED_BASES:
                if (abs(landed_base[0] - actual_x) < 1.0 and abs(landed_base[1] - actual_y) < 1.0):
                    already_landed_flag = True
                    print("BASE ALREADY LANDED!")
                    break

            if (already_landed_flag == False):                
                call_land(actual_x, actual_y, 0.2)
                print("Landed bases' positions:")
                print(LANDED_BASES)

        else:
            actual_x = DISCOVERING_POSITIONS[i][0]
            actual_y = DISCOVERING_POSITIONS[i][1]
            actual_z = FLYING_ALTITUTE_DISCOVERING

        if len(LANDED_BASES) == 3:
            break 

    print("ALL THREE LAND BASES LANDED...GOING TO HANGING BASE 2")
    target_x = HANGING_BASE_2_POSITION[0]
    target_y = HANGING_BASE_2_POSITION[1]
    target_z = FLYING_ALTITUTE_HANGING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)

    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)
    ############################################
    #IMPLEMENTA FIND BASE PRA AJUSTA CHEGADA
    #########################################
    call_land(target_x, target_y, HANGING_BASE_2_POSITION[2])

    actual_x = HANGING_BASE_2_POSITION[0]
    actual_y = HANGING_BASE_2_POSITION[1]
    actual_z = FLYING_ALTITUTE_HANGING
    print("Landed bases' positions:")
    print(LANDED_BASES)
    

    print("FOUR BASES LANDED...GOING TO HANGING BASE 1")
    target_x = HANGING_BASE_1_POSITION[0]
    target_y = HANGING_BASE_1_POSITION[1]
    target_z = FLYING_ALTITUTE_HANGING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)

    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)
    ############################################
    #IMPLEMENTA FIND BASE PRA AJUSTA CHEGADA
    #########################################
    call_land(target_x, target_y, HANGING_BASE_1_POSITION[2])

    actual_x = HANGING_BASE_1_POSITION[0]
    actual_y = HANGING_BASE_1_POSITION[1]
    actual_z = FLYING_ALTITUTE_HANGING
    print("Landed bases' positions:")
    print(LANDED_BASES)    


    print("ALL BASES LANDED.. GOING BACK TO COASTAL BASE...")
    target_x = COASTAL_BASE_POSITION[0]
    target_y = COASTAL_BASE_POSITION[1]
    target_z = FLYING_ALTITUTE_HANGING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)

    call_land(target_x, target_y, COASTAL_BASE_POSITION[2])
    print("Landed bases' positions:")
    print(LANDED_BASES)   

    print("FINISHED!!!!")


def mission_planner_sensoring_task():  
    setup_vehicle()

    print("GOING TO THE FIRST CORNER OF THE CAN....")
    actual_x = COASTAL_BASE_POSITION[0]
    actual_y = COASTAL_BASE_POSITION[1]
    actual_z = COASTAL_BASE_POSITION[2]    
    target_x = CAN_POSITIONS[0][0]
    target_y = CAN_POSITIONS[0][1]
    target_z = FLYING_ALTITUTE_DISCOVERING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)

    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)

    print("SENSORING THE CAN....")
    activate_sensor.publish(True)
    actual_x = CAN_POSITIONS[0][0]
    actual_y = CAN_POSITIONS[0][1]
    actual_z = FLYING_ALTITUTE_DISCOVERING
    target_x = CAN_POSITIONS[1][0]
    target_y = CAN_POSITIONS[1][1]
    target_z = FLYING_ALTITUTE_SENSORING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)

    print("SENSORING DONE... GOING BACK TO COASTAL BASE")
    # Avoid collision with the hanging base 1
    activate_sensor.publish(False)
    actual_x = CAN_POSITIONS[1][0]
    actual_y = CAN_POSITIONS[1][1]
    actual_z = FLYING_ALTITUTE_SENSORING   
    target_x = CENTER_ARENA_POSITION[0]
    target_y = CENTER_ARENA_POSITION[1]
    target_z = FLYING_ALTITUTE_DISCOVERING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)

    # Go home then
    actual_x = CENTER_ARENA_POSITION[0]
    actual_y = CENTER_ARENA_POSITION[1]
    actual_z = FLYING_ALTITUTE_DISCOVERING   
    target_x = COASTAL_BASE_POSITION[0]
    target_y = COASTAL_BASE_POSITION[1]
    target_z = FLYING_ALTITUTE_DISCOVERING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)

    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)
    
    # Land
    call_land(target_x, target_y, COASTAL_BASE_POSITION[2]+OFFSET_Z)

    print("FINISHED!!!!")

def mission_planner_inspection_task():  
    setup_vehicle()

    print("GOING OVER THE HANGING BASE 1....")
    actual_x = COASTAL_BASE_POSITION[0]
    actual_y = COASTAL_BASE_POSITION[1]
    actual_z = COASTAL_BASE_POSITION[2]  
    target_x = HANGING_BASE_1_POSITION[0]
    target_y = HANGING_BASE_1_POSITION[1]
    target_z = FLYING_ALTITUTE_HANGING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # Setting the target
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)
    print("APPROACHING BASE....")
    goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_1_POSITION[2], 5)
    print("INSPECTING....")
    activate_inspector.publish(True)
    time.sleep(INSPECTION_WAIT_TIME)
    activate_inspector.publish(False)
    print("GOING UP....")
    goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_1_POSITION[2], target_x, target_y, target_z, 5)

    print("GOING OVER THE FIRST BASE....")
    actual_x = HANGING_BASE_1_POSITION[0]
    actual_y = HANGING_BASE_1_POSITION[1]
    actual_z = FLYING_ALTITUTE_HANGING    
    target_x = FIXED_BASES_POSITION[0][0]
    target_y = FIXED_BASES_POSITION[0][1]
    target_z = FLYING_ALTITUTE_HANGING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # Setting the target
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)
    print("APPROACHING BASE....")
    goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING, 5)
    print("INSPECTING....")
    activate_inspector.publish(True)
    time.sleep(INSPECTION_WAIT_TIME)
    activate_inspector.publish(False)
    print("GOING UP....")
    goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING, target_x, target_y, target_z, 5)

    print("GOING OVER THE SECOND BASE....")
    actual_x = FIXED_BASES_POSITION[0][0]
    actual_y = FIXED_BASES_POSITION[0][1]
    actual_z = FLYING_ALTITUTE_DISCOVERING  
    target_x = FIXED_BASES_POSITION[1][0]
    target_y = FIXED_BASES_POSITION[1][1]
    target_z = FLYING_ALTITUTE_DISCOVERING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # Setting the target
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)
    print("APPROACHING BASE....")
    goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING, 5)
    print("INSPECTING....")
    activate_inspector.publish(True)
    time.sleep(INSPECTION_WAIT_TIME)
    activate_inspector.publish(False)
    print("GOING UP....")
    goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING, target_x, target_y, target_z, 5)

    print("GOING OVER THE THIRD BASE....")
    actual_x = FIXED_BASES_POSITION[1][0]
    actual_y = FIXED_BASES_POSITION[1][1]
    actual_z = FLYING_ALTITUTE_DISCOVERING  
    target_x = FIXED_BASES_POSITION[2][0]
    target_y = FIXED_BASES_POSITION[2][1]
    target_z = FLYING_ALTITUTE_DISCOVERING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # Setting the target
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)
    print("APPROACHING BASE....")
    goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING, 5)
    print("INSPECTING....")
    activate_inspector.publish(True)
    time.sleep(INSPECTION_WAIT_TIME)
    activate_inspector.publish(False)
    print("GOING UP....")
    goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING, target_x, target_y, target_z, 5)

    print("GOING OVER THE HANGING BASE 2....")
    actual_x = FIXED_BASES_POSITION[2][0]
    actual_y = FIXED_BASES_POSITION[2][1]
    actual_z = FLYING_ALTITUTE_DISCOVERING 
    target_x = HANGING_BASE_2_POSITION[0]
    target_y = HANGING_BASE_2_POSITION[1]
    target_z = FLYING_ALTITUTE_HANGING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # Setting the target
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)
    print("APPROACHING BASE....")
    goto_to_target_position(target_x, target_y, target_z, target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_2_POSITION[2], 5)
    print("INSPECTING....")
    activate_inspector.publish(True)
    time.sleep(INSPECTION_WAIT_TIME)
    activate_inspector.publish(False)
    print("GOING UP....")
    goto_to_target_position(target_x, target_y, FLYING_ALTITUTE_INSPECTING+HANGING_BASE_2_POSITION[2], target_x, target_y, target_z, 5)

    print("GOING HOME....")
    actual_x = HANGING_BASE_2_POSITION[0]
    actual_y = HANGING_BASE_2_POSITION[1]
    actual_z = FLYING_ALTITUTE_HANGING 
    target_x = COASTAL_BASE_POSITION[0][0]
    target_y = COASTAL_BASE_POSITION[0][1]
    target_z = FLYING_ALTITUTE_HANGING
    steps = int(math.sqrt((target_x-actual_x)**2 + (target_y-actual_y)**2)/MAX_STEP_SIZE)
    # Setting the target
    goto_to_target_position(actual_x, actual_y, actual_z, target_x, target_y, target_z, steps)

    call_land(target_x, target_y, COASTAL_BASE_POSITION[2]+OFFSET_Z)

    print("Finished!!")

set_point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
activate_sensor = rospy.Publisher("/hydrone_mission_planner/sensor/activate", Bool, queue_size=10)
activate_inspector = rospy.Publisher("/hydrone_mission_planner/gas_detector/activate", Bool, queue_size=10)

set_point_sim_pub = rospy.Publisher("/hydrone/command/pose", PoseStamped, queue_size=10)

if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    rospy.Subscriber("/mavros/state", State, mode_callback)

    rospy.spin()