## Duckietown Source Code Notes
This document is still a work in progress...
I am trying to go through each package to see what they contain.

-----

*  adafruit_drivers
 
   packages for the HATS, as well as several other componenets they seem to support using. 

*  adafruit_imu
  
   similar package for an optional IMU

*  apriltags_ros
   
   an implementation of AprilTags for ROS  
   April tags are the same as AR tags or Alvar Tags... just a different implementation.  
   Developed by Prof Edwin Olson of University of Michigan ( http://april.eecs.umich.edu )  
   Code was ported to C++ ( http://wiki.tekkotsu.org/index.php/AprilTags )  
   Code has now been modified (by MIT) to be a standalone library with homography based on openCV.   


  *  apriltags
 
    The c++ library implementation  
    The tag "families"  

  *  apriltags_ros
      
     The ROS node implementation (written in cpp)  
     Adds
     * apriltag\_detector\_node
     * apriltags\_postprocessing\_node

*   _attic
	
    Old code they didn't want to get rid of but is no longer in use...

*   car_supervisor

*   dagu_car
    
    The forward and inverse kinematics for the car.  
    This gives the following nodes  
    * forward\_kinematics\_node.py - estimate the velocity and orientation from the wheel command
    * inverse\_kinematics\_node.py - map the car command into a wheel command 
    * velocity\_to\_pose\_node.py - convert the velocity and omega to a pose. 
    