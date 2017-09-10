import cv2
import argparse
import numpy as np
import threading
import time
from hampy import detect_markers
import warnings

class Camera(threading.Thread):
    def __init__(self, camera_id = 0, video_mode = False, ducky = [None, None] ,duckybot = [None, None], obstacle = [None, None], constant_update = True):
        warnings.simplefilter("ignore")

        threading.Thread.__init__(self)
        self.thread_id = 1

        self.ID = camera_id
        self.ducky_tag = ducky
        self.duckybot_tag = duckybot
        self.obstacle_tag = obstacle
        
        # Should the program get data continuously or only when requested?
        # Set to false if you have limited cpu resources
        self.constant_update = constant_update

        # Intrinsic Parameters
        self.camMatrix = np.zeros((3, 3),dtype=np.float64)
        self.camMatrix[0][0] = 810.06435
        self.camMatrix[0][2] = 325.39548
        self.camMatrix[1][1] = 810.75645
        self.camMatrix[1][2] = 249.01798
        self.camMatrix[2][2] = 1.0

        self.distCoeff = np.zeros((1, 5), dtype=np.float64)
        self.distCoeff[0][0] = 0.01584
        self.distCoeff[0][1] = 0.37926
        self.distCoeff[0][2] = -0.00056
        self.distCoeff[0][3] = 0.00331
        self.distCoeff[0][4] = 0.0

        # Initial Capture Data Values
        self.img = None
        self.Ducky_Pose = [None, None]
        self.Duckybot_Pose = [None, None]
        self.Obstacle_Pose = [None, None]

        self.play_video = video_mode

        self.initialize()


    def initialize(self):
        # Initialize video capture
        self.cap = cv2.VideoCapture(self.ID)

        frameRate = 20.0
        frameWidth = 640
        frameHeight = 480

        if cv2.__version__[0] == "2":
            # Latest Stable Version (2.x)
            self.cap.set(cv2.cv.CV_CAP_PROP_FPS, frameRate)
            self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, frameWidth)
            self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, frameHeight)
        else:
            # version 3.1.0 (BETA)
            self.cap.set(cv2.CAP_PROP_FPS, frameRate)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameWidth)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)

        self.thresh = 0.4
        self.thresh_img = np.zeros((frameHeight, frameWidth, 3), dtype=np.uint8)

    def release(self):     
        self.is_stopped = True
        time.sleep(1)
        # get rid of video stream window
        if self.play_video:
            cv2.destroyWindow('live')
        # Release video capture
        self.cap.release()


    def __str__(self):
        # String representation of this camera
        output = "===========CAMERA INFORMATION===========\n"
        output += "Camera Device ID: " + str(self.ID)
        output += "\n\nIntrinsic Parameters: \n" + str(self.camMatrix) + "\n"
        output += "\nRegistered AR Tags:"
        if self.ducky_tag != [None, None]:
            output += "\nDucky:    \t ID: {:10} \t Size: {:3}mm".format(self.ducky_tag[0], self.ducky_tag[1])
        if self.duckybot_tag != [None, None]:
            output += "\nDuckybot: \t ID: {:10} \t Size: {:3}mm".format(self.duckybot_tag[0], self.duckybot_tag[1])
        if self.obstacle_tag != [None, None]:
            output += "\nObstacle: \t ID: {:10} \t Size: {:3}mm".format(self.obstacle_tag[0], self.obstacle_tag[1])
        output += "\n========================================\n"
        return output



    def run(self):
        self.is_stopped = False
        while not self.is_stopped:
            # Get Data as often as possible if playing video or set to constantly update
            if self.constant_update or self.play_video:
                self.capture_data()

            # Show Video
            if self.play_video:
                if self.img is not None:
                    cv2.imshow('live', self.img)
                    #cv2.imshow('processed', self.thresh_img)
                    if cv2.waitKey(1) & 0xFF == ord('p'):
                        cv2.imwrite('out.jpg', self.img)


    def capture_data(self):

        # Get Frame
        okay, self.img = self.cap.read()
        if self.img is None:
            # Bad Image, do nothing
            return


        # convert image to grayscale then back so it still has
        # 3 color dimensions
        gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)

        # threshold the image to either pure white or pure black
        # thresh is scaled so that images is thresholded at % out of 255
        self.thresh_img[:, :, :] = 255.0*np.round(gray_img[:, :, :]/(510.0*self.thresh))

        # blur the rounded image
        #blurred_image = cv2.GaussianBlur(self.thresh_img,(5,5),0)

        # find any valid AR tags in the image
        markers = detect_markers(self.thresh_img)

        # Assume no objects are available until proven otherwise
        ducky_unavailable = True
        duckybot_unavailable = True
        obstacle_unavailable = True

        # for each valid tag, get the pose
        for m in markers:
            # Draw the marker outline on the image
            m.draw_contour(self.img)

            # Label the tag ID on the image
            if cv2.__version__[0] == "2":
                # Latest Stable Version
                cv2.putText(self.img, str(m.id), tuple(int(p) for p in m.center), cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            else:
                # version 3.1.0 (Dylans Version)
                cv2.putText(self.img, str(m.id), tuple(int(p) for p in m.center), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

            # Get ducky pose if ducky AR tag was detected
            if (m.id == self.ducky_tag[0]):
                self.Ducky_Pose = self.get_object_pose(m, self.ducky_tag)
                ducky_unavailable = False

            # Get duckybot pose if duckybot AR tag was detected
            elif (m.id == self.duckybot_tag[0]):
                self.Duckybot_Pose = self.get_object_pose(m, self.duckybot_tag)
                duckybot_unavailable = False

            # Get obstacle pose if obstacle AR tag was detected
            elif (m.id == self.obstacle_tag[0]):
                self.Obstacle_Pose = self.get_object_pose(m, self.obstacle_tag)
                obstacle_unavailable = False
            elif (self.obstacle_tag[0] == -100):
                # Obstacle tag is in return any mode, get this unknown tag and set it to obstacle pose
                self.Obstacle_Pose = self.get_object_pose(m, self.obstacle_tag)
                obstacle_unavailable = False                

        # set poses for objects not found
        if ducky_unavailable:
            self.Ducky_Pose = [None, None]
        if duckybot_unavailable:
            self.Duckybot_Pose = [None, None]
        if obstacle_unavailable:
            self.Obstacle_Pose = [None, None]

        # Finished capturing available data, return
        return


    # Given a matching marker and tag, get the pose
    def get_object_pose(self, marker, tag):
        # AR Tag Dimensions
        objPoints = np.zeros((4, 3), dtype=np.float64)
        objPoints[0,0] = -1.0*tag[1]/2.0
        objPoints[0,1] = tag[1]/2.0
        objPoints[0,2] = 0.0
        objPoints[1,0] = tag[1]/2.0
        objPoints[1,1] = tag[1]/2.0
        objPoints[1,2] = 0.0
        objPoints[2,0] = tag[1]/2.0
        objPoints[2,1] = -1*tag[1]/2.0
        objPoints[2,2] = 0.0
        objPoints[3,0] = -1*tag[1]/2.0
        objPoints[3,1] = -1*tag[1]/2.0
        objPoints[3,2] = 0.0

        # Get each corner of the tags
        imgPoints = np.zeros((4, 2), dtype=np.float64)
        for i in range(4):
            imgPoints[i, :] = marker.contours[i, 0, :]


        camPos = np.zeros((3, 1))
        camRot = np.zeros((3, 1))

        # SolvePnP
        retVal, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camMatrix, self.distCoeff)
        Rca, b = cv2.Rodrigues(rvec)
        Pca = tvec

        return [Pca, Rca]

    def get_all_poses(self):
        # If the program is set to not automatically capture data, capture it now
        if not self.constant_update and not self.play_video:
            self.capture_data()     
        # Return up to date data
        return (self.Ducky_Pose, self.Duckybot_Pose, self.Obstacle_Pose)
