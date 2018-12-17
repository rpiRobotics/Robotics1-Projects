# Aruco Test from camera
import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

#import pandas as pd
from matplotlib.lines import Line2D
import cv2
#img = cv2.imread('tiles_226.png')
#cv2.namedWindow('ImageWindow',cv2.WINDOW_NORMAL)
#cv2.imshow('ImageWindow',img)

#cv2.resizeWindow('ImageWindow', 600,600)
#cv2.waitKey(0)
#if cv2.waitKey(1) & 0xFF == ord('q'):

   # cv2.destroyAllWindows()

cap = cv2.VideoCapture('test1.mp4')

while(True):
    ret, frame = cap.read()
    if ret == True:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('frame', gray)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

