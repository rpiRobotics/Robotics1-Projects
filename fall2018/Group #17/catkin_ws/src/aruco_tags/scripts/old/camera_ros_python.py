#!/usr/bin/env python
# Camera bit for the simple aruco test while I don't have howard with me

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2, PIL
from cv2 import aruco
from cv_bridge import CvBridge

def talker():
    pub = rospy.Publisher('/howard17/camera_node/image/compressed', CompressedImage, queue_size=10)
    rospy.init_node('aruco_talker')
    rate = rospy.Rate(10) # 10hz
    br = CvBridge()
    #cap = cv2.VideoCapture('test1.mp4')
    cap = cv2.VideoCapture(0)
    #print('Got the cap')
    #cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret == True:
	    
            #print('ret is true')
            # Our operations on the frame come here
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Display the resulting frame
            #cv2.imshow('frame',gray)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #break
            #rospy.loginfo(hello_str)

	    # Convert the video to a ros Compressed image
	    message = br.cv2_to_compressed_imgmsg(frame)
            pub.publish(message)
	    #rospy.loginfo(message)
        #rate.sleep()
    # Upon shutdown
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
