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
FLYING_ALTITUTE_DISCOVERING = 2.5
FLYING_ALTITUTE_SENSORING = 1.0
FLYING_ALTITUTE_INSPECTING = 0.5

DISCOVERING_POSITIONS = [   [8.0, 8.0], [5.0, 5.0], [2.0, 2.0], [5.0, 2.0]   ]

COASTAL_BASE_POSITION = [8.0, 2.0, 0.5]
HANGING_BASE_1_POSITION = [5.0, 2.0, 1.7]
HANGING_BASE_2_POSITION = [8.0, 8.0, 1.3]
CENTER_ARENA_POSITION = [5.0, 5.0]

INSPECTION_WAIT_TIME = 12

FIXED_BASES_POSITION = [ [5.0, 2.0], [8.0, 8.0], [4.0, 4.0] ]

LANDED_BASES = []

PX_PY_THRESHOLD = 20
ADJUSTED_STEP = 0.01

CAN_POSITIONS = [[0.25, -6.45], [6.15, 0.45]]

set_point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
activate_sensor = rospy.Publisher("/hydrone_mission_planner/sensor/activate", Bool, queue_size=10)
activate_inspector = rospy.Publisher("/hydrone_mission_planner/gas_detector/activate", Bool, queue_size=10)

set_point_sim_pub = rospy.Publisher("/hydrone/command/pose", PoseStamped, queue_size=10)