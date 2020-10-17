#! /usr/bin/env python
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
import sensors_class as sc
import cv2
import time, os, math
import numpy as np
from cv_bridge import CvBridge

mark = sc.Sensors()
bridge_ = CvBridge()

flag = False
pub = rospy.Publisher('/sensor_detection', String, queue_size=10)
pub_end = rospy.Publisher('/sensor_detection_finished', Bool, queue_size=10)

total_detected = 0
flag_detection = False
detections_to_flag = 5
counter_false_detection = 0

def activate_sensor_detection(data):
    global flag
    if data.data == True:
        print("Detection Enabled!")
        flag = True
    else:
        print("Detection Disabled!")
        flag = False

def image_callback(data):
    global flag
    global total_detected
    global detections_to_flag
    global counter_false_detection
    global flag_detection

    max_detections = 30

    if flag == True:
        if total_detected < max_detections:
            cvim = bridge_.imgmsg_to_cv2(data, "bgr8")
            cvim = cvim[:,-400:-200]
            # print(cvim.shape)
            mark.setImage(cvim, 0.)
            # print(type(data))
            
            # get reference frames
            R1, T1, success1, R2, T2, success2 = mark.getRefFrameSMarks()

                  
            
            if (success1 == True and total_detected < max_detections and flag_detection == True):   
                # print("BAD Sensor Detected!")
                rospy.loginfo("BAD Sensor Detected!")              	
                pub.publish("BAD")

                total_detected += 1 

            if success1 == False:
                counter_false_detection += 1
            else:
                counter_false_detection = 0

            if (counter_false_detection >= detections_to_flag):
                flag_detection = True                
            else:
                flag_detection = False      
            
            # show
            mark.show()
            cv2.waitKey(30)
        else:
        #     rospy.loginfo("All 3 bad sensors were detected!")
            pub_end.publish(True)
    else:
        cv2.destroyAllWindows()


if __name__ == "__main__": 
    rospy.init_node("sensors_detector", anonymous=False)    

    image = rospy.Subscriber("/hydrone/camera_camera/image_raw", Image, image_callback, queue_size = 1)
    rospy.Subscriber("/sensor_detector/activate", Bool, activate_sensor_detection)

    rospy.spin()
