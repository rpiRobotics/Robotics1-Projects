##Team 6 / Catchie Duckies  
###Kimberly Oakes, YC Peng, Yanshan He  

The dobot_RR files were modified so that the setJointPositions command also takes the input of the gripper angle.

fkdobot.m is a function which takes inputs q1, q2, and q3 and generates the vector p0g which is the x,y,z of the gripper end point.

ikdobot.m is a function which takes inputs x,y,z of the gripper end point and returns the vector Q containing q1,q2,q3 and an error message.

keycontrol_joints.m
This program allows you to input the desired keys to control each joint angle. The code is currently set so that the three main joints change by 3 degrees with each key press. The rotation and gripper change by 5 degrees.

keycontrol2.m
This program allows you to input the desired keys to control the x,y,z of the gripper as well as the rotation and gripper angles. The program is currently set to change the x and y by 10mm, z by 2 mm, and the rotation and gripper by 5 degrees.

Main_v20161204_MultiDuck
The main file is Visual_Servoing_task3_2ducks. It requires the other files in the folder. It detects two ducks and will grab whichever one is covered.

Main_v20161204-20161204T234004Z
follow_duck.m recognizes the duck at different heights and moves to the position. Popup boxes appear between movements to give the user time to move duck.

kalman.m
This file predicts the location of the duck at a future time after taking 15 images. After the images have been taking, the arm moves to the predicted position. It uses LocateDuckieDR, imgBG, and ikdobot. A popup window appears so that the arm can move to its initial position before starting to collect images.

obstacle_avoidancev2.m
This file goes around a pre-defined object using potential field method. It requires ikdobot. This file also has a popup to delay the arm moving. A figure appears which shows the obstacle in black and the path generated.

pathfollow.m
The arm starts at a place within a circle and merges onto a half circle path. Figure is generated showing desired and actual path. Popup is included.
