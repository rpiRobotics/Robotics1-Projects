#!/usr/bin/env python
# Simple ros Aruco example. Subscribes to the camera and does stuff
import rospy
import sys
import time
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
#from cv2 import aruco

def callback(data):
    br = CvBridge()
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #rospy.loginfo("Received camera stream");
    frame = br.compressed_imgmsg_to_cv2(data)    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('frame', gray)
    
    #___________ Aruco Stuff ____________


    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    # Clear the previously drawn plot
    plt.clf()

    # Plot the detections
    plt.imshow(frame_markers)
    
    # if anything was detected, plot the dots
    if ids is not None: 
        for i in range(len(ids)):
            c = corners[i][0]
            
            # Plot blue dots for id = 1
            if ids[i] == 1:
                plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "bo") #, label = ids[i])
            # Plot yellow dots for id = 2
            elif ids[i] == 2:
                plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "yo") #, label = ids[i])
            # Plot green dots for any other ids
            else:
                plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "go") #, label = ids[i])
                
    # Doesn't work without this. especially pressing q doesnt work
    # Press q a lot of times
    plt.pause(0.05)


    #__________End of Arucuo Stuff ______
    # Need these lines or it wont work!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cvshutdown()

    
def listener():
    br = CvBridge()
    print('Running listener')
    rospy.init_node('aruco_listener')
    #rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('/howard17/camera_node/image/compressed', CompressedImage, callback)
    rospy.spin()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cvshutdown()
    
 

def cvshutdown():
    cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()
    rospy.on_shutdown(cvshutdown)
    
    # launch the camera: roslaunch pi_camera camera_node.launch veh:=howard17
    # rosrun aruco_tags aruco_ros_python.py
    
