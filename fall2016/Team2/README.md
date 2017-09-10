# RobotArm
F16 Robotics I Project

Team 2: Double Ducker

Members:
Christopher Ho,
Mitchell Phillips,
Jacob Song

The following MATLAB Scripts were used throughout the duration of the Fall 2016, Robotics I Project for the Trossen PhantomX Articulated Arm.

Control of the PhantomX was done so using Robot Raconteur and the associated python servers.

Requires the Robot Raconteur Image Processing Server.

Utilized the desktop servers that were available in the lab.

The final design and demonstration used the visualServo_Duckie.m file. 

It was intended to replicate the sorting of two different colored ducks by sorting two ARTags and placing them either left or right of the arm. 

The contents for the project are explained below.

Note: Project is dependent on Professor Wen’s subproblem code’s and common robotics transformations, 
Peter Corke’s RVC Tools, Robot Raconteur, and MATLAB’s Image Processing Toolbox. Adjust MATLAB File path as needed.

Majority of scripts require the use of phantomX_Inital.m as it initializes the PhantomX Structure and the connection to the arm. 

--------------------------------------------------------------------------------------------------------------------------------------------------


	visualServo_Duckie.m
		Sorts two ARTags to represent two different colored ducks.
		Uses: 	
	
		-	phantom_Inital.m
		-	camera_Initial_Duckie.m
		-	arDetect.m
		-	locPos.m
		-	smoothMotion.m
		-	duckieDrop.m
		-	duckieGrab.m
		-	Home.m

--------------------------------------------------------------------------------------------------------------------------------------------------

MOTION;
	
	CircleMotion:
		-	Scripts for generating and analyzing a planned path of a circle

	WenProvided:
		-	Includes common robotics transformations and subproblems

	delay.m
		Delay function similar to pause.

	duckieDrop.m
		Drops the already grasped duck by opening the grabber at the end-effector.

	duckieGrab.m
		Closes the grabber at the end-effector in order to grasp a duck.

	Fwdkinstep.m
		Iterative computation of the forward kinematics. Initially provided by Professor Wen.

	Home.m
		Sets the phantomX to its zero configuration.
		Uses:
		-	phantomX_ForwardKinematics.m
		-	smoothMotion.m

	Lowpass.m
		Generates a lowpass filter for smooth motion of the arm. Implemented after presentations.

	phantomX_ForwardKinematics.m
		Computes the forward kinematics for the 4DOF PhantomX Robot Arm
		Uses:
		-	fwdkinstep.m

	phantomX_Inital.m
		Initializes the phantomX structure and connection to the arm.

	phantomX_InverseKinematics_JS.m
		Computes the inverse kinematics for the 4DOF PhantomX Robot Arm. Was used for final design.

	phantomX_InverseKinematics_MP.m
		Alternative Version for inverse kinematics. Not used for final design.

	radstep.m
		Converts angle in radians to stepper motor units.

	smoothMotion.m
		Follows the generated trapezoidal trajectory path and executes the steps iteratively
		Uses:	
		-	trapezoidal_generation_v2.m

	smoothMotionLP.m
		Follows the generated trapezoidal trajectory path. Implements low pass filter
		Uses:	
		-	Lowpass.m
		-	trapezoidal_generation_v2.m
	
	step2rad.m
		Converts stepper motor units to angle in radian for kinematic computations.
	
	trapezoidal_generation_v2.m
		Generates a trapezoidal trajectory, s-curve, path for the arm to follow.

--------------------------------------------------------------------------------------------------------------------------------------------------

SIMULATION;

	-  Contains scripts for collision detection simulation and path diagnostics/ troubleshooting.

--------------------------------------------------------------------------------------------------------------------------------------------------
			
VISION;

	ARTags:

		arDetect.m
			Generates the homogeneous transform for the ARTag.
			Uses:
			-	camera_Inital.m

		camera_Initial.m
			Initializes the camera object and connects to the camera. 
			Enables the use of ARTags and connects to the Robot Raconteur image processing server.
		
		camera_Inital_Duckie.m
			Similar to camera_Inital. Uses multiple ARTags. Used for final design.
		
		locPos.m
			Generates the position vector from the inertial frame to the task.
			Uses:
			-	arDetect.m
		
		visualServo_AR.m
			Moves arm to the ARTag.
			Uses:
			-	phantomX_Inital.m
			-	camera_Initial.m

		visualServo_AR_Follow.m
			Similar to visualServo_AR.m. Will continue to move to a position above an ARTag.
			Uses:
			-	phantomX_Inital.m
			-	camera_Inital.m


	ImageThresholding:
		
		Masks:
		-	Contains functions for the different color masks implemented. 
			Each were created using the Color Thresholding Application.

		acquireObject.m
			Main. Identifies a colored duck and provides a visual feedback. 
			Not a finished product. PnP problem attempt is currently commented out.
		
		BoundBox.m
			Places a bounding box around the masked image.

		cannyTest.m
			Produces a canny edge image.

		duckDetect.m
			Detects the largest masked cluster and identifies that as the object of interest.

		duckPnp.m
			PnP problem attempt. Uses Corke’s RVC Tools Pnp function.

		idBox.m
			Creates a new bounding box around the largest masked clusted. Used for the Pnp problem attempt.
	
		pixelFilter.m
			Removes small masked binary clusters.

		References.m
			Not a legitimate m-file. Consists of a series of comments with URLs for various sources. 
			Image Thresholding code would not have been possible without these.
		
