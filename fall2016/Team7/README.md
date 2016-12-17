# Robotics I Project #
* Team #7, DEPHASE
* Dylan Elliot, Peter Horak, Andrew Showers

### Interfaces ###

* **MainCode.py** - starts the main user interface
    * Need to set constants such as CAMERA_ID, DUCKY (AR tag #), etc. based on your setup
    * Update line 415 to pass the serial port for the Dobot on your machine to the serial interface (e.g. /dev/ttyACM0 on Linux)
* **keyboard_control.py** - control the Dobot manually using your keyboard
    * pass the serial port for the Dobot with the command-line flag "-p $PORT" (e.g. PORT=/dev/ttyACM0 on Linux)

### Contents ###
* **MainCode.py** - high-level behavioral code
* **keyboard_control.py** - command-line interface to control the Dobot
* Serial Interface
    * **Controller.py** - wrapper class for the serial interface used by keyboard_control.py
    * **SerialInterface.py** - class representing the Dobot serial interface (from pyDobot)
    * **StatusMessage.py** - message class used by SerialInterface.py (from pyDobot)
* Camera
    * **AR_Camera.py** - wrapper class for the webcam and AR tag detection
    * **campose.py** - provodes a routine to estimate the camera-end effector offset
* Path Planning
    * **Roadmap.py** - class implementing a probabilistic roadmap for path planning
    * **Simulation.py** - wrapper class to store obstacles and check for collisions
    * **intersect.py** - this module implements triangle-triangle intersection tests
    * **DobotMode.py** - this module handles the Dobot kinematics
    * **math3D.py** - this module calculates rotation matrices
* Examples
    * **roadmap_test.py** - example code for using the probabilistic roadmap
    * **calibration_test.py** - example code for using campose.py
    * **arm_cal_test.py** - partial analysis of data from arm calibration


### Dependencies ###

* cv2, hampy
* numpy (1.11.2 or higher)
* scipy (spicy.spatial.kdtree, scipy.linalg)
* networkx
* matplotlib (pyplot)
* mpl_toolkits (mplot3d)
* curses
* warnings, collections, binascii, argparse, struct, itertools
* datetime, serial, time, sys, os.path, thread, threading

### Tested Environments ###

* Windows 10, Python 2.7.12, OpenCV 3.1.0
* Ubuntu 14.04, Python 2.7.12, OpenCV 3.1.0
* Lubuntu 14.04, Python 2.7.6, OpenCV 2.4.13
* OS X 10.11.6, IPython 2.7.10, w/o camera

### Unit Tests ###

* DobotModel.test()
* intersect.test()
* campose.test()

