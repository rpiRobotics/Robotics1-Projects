import AR_Camera
import time
import sys
import threading
import numpy as np
import scipy.linalg as linalg
import DobotModel
import SerialInterface
import Simulation
import Roadmap

#=====DEFINED CONSTANTS=====
CAMERA_ID = 0
VIDEO_MODE = True
MAXHEIGHT = 115 # Dobot Max Z Value

# AR TAGS
DUCKY = [12345678, 25]
DUCKYBOT = [16273625, 25]
OBSTACLE = [-100, 25.65] # [ -100, SIZE] returns any tag not equal to DUCKY/DUCKYBOT
REGISTERED_TAGS = [DUCKY, DUCKYBOT, OBSTACLE]

# FIXED POSITIONS
CAMERA_OFFSET = [[-9.059], [-24.953], [30.019]]
DUCKY_POS = np.reshape(np.array([149.66,-228.8,-1]) ,(3, 1))
GARBAGECAN_POS = np.reshape(np.array([0,-228.5,MAXHEIGHT]) ,(3, 1))


OBS_TABLE = np.array([[[70,-345,10],[70,345,10],[345,345,10]], \
    [[70,-345,10],[345,-345,10],[345,345,10]]])
OBS_POLL = np.array([[[235.0,-200,10],[235,-160,10],[235,-160,400]], \
    [[235,-200,10],[340,-200,10],[235,-160,400]], \
    [[235,-160,10],[340,-160,10],[235,-160,400]], \
    [[340,-200,10],[340,-160,10],[235,-160,400]]])
OBSTACLES = [OBS_TABLE, OBS_POLL]


#COMMENT OUT WHEN THERE ARE NO OBSTACLES (DISABLE PATH PLANNING)
#sim = Simulation.Simulation()
#for obs in OBSTACLES:
#    sim.add_obstacles(obs)
#PRM = Roadmap.Roadmap(sim,100,np.array([[-75,75],[0,60],[0,60]]))
PRM = None # if not using roadmap


# Display poses of all objects
# [ [Vector tag to camera] ,  [Rotation of tag] ]
# If not present, [None, None] will be reported
def get_data(camera):
    print "----------------------------"
    print "Ducky Pose:", camera.Ducky_Pose
    print "Duckybot Pose:", camera.Duckybot_Pose
    print "Obstacle Pose:", camera.Obstacle_Pose
    print "----------------------------"


# FUNCTION: move_xyz - Move the end effector to a desired XYZ position
def move_xyz(interface, target, pump_on = False, joint_4_angle = 0, path_planning = False):
    # Joint angles needed to reach target XYZ position
    angles = DobotModel.inverse_kinematics(target)
    # No solution was found
    if any(np.isnan(angles)):
        print "Error: No solution for coordinates: ", target
    # Solution found and path planning is disabled
    elif not path_planning:
	# Send serial command to change the joint angles
        interface.send_absolute_angles(float(angles[0]),float(angles[1]),float(angles[2]), joint_4_angle, interface.MOVE_MODE_JOINTS, pump_on)
    # Solution found and path planning is enabled
    else:
	# Find a feasible path to the desired target
        start = interface.current_status.angles[0:3]
        path = PRM.get_path(start, angles)
	# No path found, future actions may cause collision! Terminate Immediately
	if path == [] or path == None:
	    print "ERROR: No Path Found! Aborting..."
	    sys.exit()
	# Path found. Iterate through the list of positions
        for p in path:
            interface.send_absolute_angles(float(p[0]), float(p[1]), float(p[2]), joint_4_angle, interface.MOVE_MODE_JOINTS, pump_on)


# FUNCTION: get_xyz - Get required XYZ to move end effector to AR tag
def get_xyz(interface, xyz_from_camera):
    angles = interface.current_status.angles[0:3]

    # Get current XYZ
    P0t = DobotModel.forward_kinematics(angles)

    # Getting Desired XYZ of end effector
    Pct = np.array(CAMERA_OFFSET)
    R0t = DobotModel.R0T(angles)
    Rtc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    R0c = np.matmul(R0t, Rtc)

    Pta = np.matmul(R0c, xyz_from_camera) - np.matmul(R0c, Pct)
    target = np.reshape(Pta, (3, 1)) + np.reshape(P0t, (3, 1))
    return target


# FUNCTION: Touch - Place the end effector on top of an AR tag
# AR TAGS: DUCKY = 0  DUCKYBOT = 1   OBSTACLE = 2
def touch(interface, tag_index, mode, pump_on = False):

    # Get the object XYZ position of the tag relative to the camera
    object_xyz = camera.get_all_poses()[tag_index][0]

    # Get the target XYZ position
    target = get_xyz(interface, object_xyz)

    # DIRECT MODE
    if mode == 1:
        # Move directly to target
        move_xyz(interface, target, pump_on)

    # ARIAL MODE
    elif mode == 2:
        # Move directly above target
        move_xyz(interface, target + np.reshape(np.array([0,0,50]) ,(3, 1)))
        # Move to target
        move_xyz(interface, target, pump_on)



# FUNCTION: Track - Follow an AR tag by hovering the camera above it.
# AR TAGS: DUCKY = 0  DUCKYBOT = 1   OBSTACLE = 2
def track(interface, camera, tag_index):
    # EVENT LISTENER
    req_exit = []
    listener = threading.Thread(target=input_thread, args=(req_exit,))
    listener.start()

    alpha = 0.2 # weight of new measurements
    P0a_est = None # estimate of AR tag position

    while not req_exit:
        # Search if the camera module fails to find the tag for 30 consecutive frames
        searching = True
        for x in range(0,30):
            data = camera.get_all_poses()[tag_index]
            if data != [None, None]:
                searching = False
                break;

        if searching:
            # Search until the desired tag is found
            data = search(interface, camera, tag_index)

        # SEARCH TERMINATED DUE TO BREAK SIGNAL
        if data == None:
            return

	# Follow tag while it is in view
	while data != [None, None] and not req_exit:
	    
	    # === Getting Desired XYZ of end effector ===
	    # Hover above the tag (Z offest 5 * marker size)
	    Pct = np.array([[0], [0], [5 * REGISTERED_TAGS[tag_index][1]]])
	    
	    # Calculate difference in desired position and actual position
	    Roc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
	    Pta = np.matmul(Roc, data[0]) - np.matmul(Roc, Pct)
	    
	    # Only move if the change is significant (1mm)
	    if np.linalg.norm(Pta) > 1:
		# Current XYZ position
		p0t = DobotModel.forward_kinematics(interface.current_status.get_angles())
		# Desired XYZ position
		target = np.reshape(Pta, (3, 1)) + np.reshape(p0t, (3, 1))
		move_xyz(interface,target)		
	    
	    # Get object pose data
	    data = camera.get_all_poses()[tag_index]    
	    
	    
	    '''
	    LOW PASS FILTER VERSION
	    # From kinematics
	    angles = interface.current_status.angles[0:3]
	    P0t = np.reshape(DobotModel.forward_kinematics(angles), (3,1))
	    R0t = DobotModel.R0T(angles)

	    # From calibration
	    Pct = np.array(CAMERA_OFFSET)
	    Rtc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
	    R0c = np.matmul(R0t,Rtc)

	    # From camera
	    Pca = np.reshape(data[0], (3,1))

	    # Hover above the tag (Z offest 5 * marker size)
	    Pca_des = np.array([[0], [0], [5 * REGISTERED_TAGS[tag_index][1]]])
	    # correction = -1*np.matmul(R0c, Pca_des - Pca)
	    P0a_des = P0t + np.matmul(R0c, Pca_des - Pct)

	    # Smoothed estimate
	    if P0a_est is None:
		P0a_est = P0a_des # initialize so no movement required
	    P0a =  P0t + np.matmul(R0c, Pca - Pct) # measured
	    P0a_est = alpha*P0a + (1 - alpha)*P0a_est # update estimate with new measurement

	    # If the change in desired XYZ is notable, move to track it
	    correction = P0a_des - P0a_est
	    if np.linalg.norm(correction) > 1.0:
		angles = move_xyz(interface, P0t - correction)

	    # Get object pose data
	    data = camera.get_all_poses()[tag_index]
	    '''


# FUNCTION: Search - Rotate base by increments of 5 degrees until the desired tag is found
# AR TAGS: DUCKY = 0  DUCKYBOT = 1   OBSTACLE = 2
# If tag_index is left undefined , search will return upon finding any registered tag.
# Cleanup mode places unknown tags in the designated trash bin.
def search(interface, camera, tag_index = -1, clean_mode = False, path_planning = False):

    # Current base angle is the starting point
    base_angle = interface.current_status.get_base_angle()
    direction = 1

    # EVENT LISTENER
    req_exit = []
    listener = threading.Thread(target=input_thread, args=(req_exit,))
    listener.start()

    # While there is no break signal, continue search
    while not req_exit:
	# Positive angle change direction
        if direction == 1:
            while base_angle < 100 :
		# BREAK SIGNAL DETECTED
                if req_exit:
                    return None

                # Get object pose data
                data = camera.get_all_poses()

                # If cleanup mode is active and garbage is detected, throw it away
                if clean_mode and data[2] != [None, None]:
		    # Position Before Cleaning
		    initial_pos = DobotModel.forward_kinematics([base_angle,10,10,0])
		    # Pick up the AR tag (garbage tag)
                    garbage_xyz = get_xyz(interface, data[2][0])
                    touch(interface, 2, 1, True)

		    # Go to max height above AR tag
                    tmp = np.zeros((3, 1))
                    tmp[:, :] = garbage_xyz[:, :]
                    tmp[2, 0] = MAXHEIGHT
                    move_xyz(interface, tmp, True)

                    # Go to garbage can
                    move_xyz(interface, GARBAGECAN_POS, True, 0, path_planning)
		    # Release Garbage
                    move_xyz(interface, GARBAGECAN_POS, False)
		    # Return to last position during searching process
		    move_xyz(interface, initial_pos, False, 0, path_planning)
                    continue

                # Search mode: Find any tag
                if tag_index == -1:
                    for x in range(0,3):
                        if data[x] != [None, None]:
                            return data[x]
		 # Search mode: Find specific tag
                else:
                    if data[tag_index] != [None, None]:
                        return data[tag_index]
		    
		# Rotate base angle by 5 degrees
                base_angle = base_angle + 5
                interface.send_absolute_angles(base_angle, 10, 10, 0)
		
	# Negative angle change direction
        else:
            while base_angle > -100 :
		# BREAK SIGNAL DETECTED
                if req_exit:
                    return None

                # Get object pose data
                data = camera.get_all_poses()

                # If cleanup mode is active and garbage is detected, throw it away
                if clean_mode and data[2] != [None, None]:
		    # Position Before Cleaning
		    initial_pos = DobotModel.forward_kinematics([base_angle,10,10,0])
		    # Pick up the AR tag (garbage tag)
                    garbage_xyz = get_xyz(interface, data[2][0])
                    touch(interface, 2, 1, True)

		    # Go to max height above AR tag
                    tmp = np.zeros((3, 1))
                    tmp[:, :] = garbage_xyz[:, :]
                    tmp[2, 0] = MAXHEIGHT
                    move_xyz(interface, tmp, True)

		    # Go to garbage can
		    move_xyz(interface, GARBAGECAN_POS, True, 0, path_planning)
		    # Release Garbage
		    move_xyz(interface, GARBAGECAN_POS, False)
		    # Return to last position during searching process
		    move_xyz(interface, initial_pos, False, 0, path_planning)
		    continue

		# Search mode: Find any tag
		if tag_index == -1:
		    for x in range(0,3):
			if data[x] != [None, None]:
			    return data[x]
		 # Search mode: Find specific tag
		else:
		    if data[tag_index] != [None, None]:
			return data[tag_index]

		# Rotate base angle by 5 degrees
		base_angle = base_angle + 5
		interface.send_absolute_angles(base_angle, 10, 10, 0)

        # change direction
        direction = direction * -1

    # user requested to exit
    interface.send_absolute_angles(0, 10, 10, 0)
    return None

# FUNCTION: place_ducky - Place the ducky onto a desired target
def place_ducky(interface, target, joint_4_angle = 0, path_planning = False):
    # Get the position we want to place the ducky
    goal_xyz = get_xyz(interface, target) + np.array([[0], [0], [43]])

    # === GETTING THE DUCKY ===
    # Move 80mm above the ducky
    ducky_xyz = DUCKY_POS + np.array([[0], [0], [80]])
    angles = DobotModel.inverse_kinematics(ducky_xyz)
    move_xyz(interface, ducky_xyz, True, -angles[0], path_planning)

    # Move directly onto the ducky with pump on
    move_xyz(interface, DUCKY_POS, True, -angles[0])

    # === MOVING DUCKY TO GOAL ===
    # Move to max height with the pump still on
    move_xyz(interface,np.reshape(np.array([149.66,-228.5,MAXHEIGHT]) ,(3, 1)), True, joint_4_angle)

    # Move to max height above the desired goal
    tmp = np.zeros((3, 1))
    tmp[:, :] = goal_xyz[:, :]
    tmp[2, 0] = MAXHEIGHT
    move_xyz(interface, tmp, True, joint_4_angle, path_planning)

    # Move to desired position with pump on
    move_xyz(interface, goal_xyz, True, joint_4_angle)

    # Release the pump
    move_xyz(interface, goal_xyz, False, joint_4_angle)

    # === DUCKY PLACED. RETURN TO ZERO CONFIG ===
    # Move back up from the drop location
    move_xyz(interface, tmp, False, joint_4_angle)
    # Move to default position
    interface.send_absolute_angles(0, 10, 10, 0)
    
    
# FUNCTION: Stack - Stack 3 tags directly on top of eachother.
# Pre-requisite: All tags must be registered (DUCKY, DUCKYBOT OBSTACLE) and be in view
def stack(interface, cam_data):
    # Calculate the drop locations
    drop_location_1 = np.reshape(get_xyz(interface, cam_data[0][0]) ,(3, 1)) + np.reshape(np.array([0,0,10]) ,(3, 1))
    drop_location_2 = np.reshape(get_xyz(interface, cam_data[0][0]) ,(3, 1)) + np.reshape(np.array([0,0,20]) ,(3, 1))
    # Calculate the pickup locations
    pickup_1 = get_xyz(interface, cam_data[1][0])
    pickup_2 = get_xyz(interface, cam_data[2][0])

    # Pickup the first tag
    move_xyz(interface, pickup_1, True)
    tmp = np.zeros((3, 1))
    tmp[:, :] = pickup_1[:, :]
    tmp[2, 0] = 0
    move_xyz(interface, tmp, True)

    # Dropoff first tag
    move_xyz(interface, drop_location_1, True)
    move_xyz(interface, drop_location_1, False)
    tmp = np.zeros((3, 1))
    tmp[:, :] = drop_location_1[:, :]
    tmp[2, 0] = 0
    move_xyz(interface, tmp, True)

    # Pickup second tag
    tmp = np.zeros((3, 1))
    tmp[:, :] = pickup_2[:, :]
    tmp[2, 0] = 0
    move_xyz(interface, tmp, True)
    move_xyz(interface, pickup_2, True)
    tmp = np.zeros((3, 1))
    tmp[:, :] = pickup_2[:, :]
    tmp[2, 0] = 0
    move_xyz(interface, tmp, True)

    # Dropoff second tag
    move_xyz(interface, drop_location_2, True)
    move_xyz(interface, drop_location_2, False)    


# USER INPUT THREAD - Used for break signals
def input_thread(usr_list):
    raw_input()
    usr_list.append(None)


if __name__ == '__main__':

    # INTITIALIZING SERIAL INTERFACE
     # 6 Refers to COM7 For Windows, Leave blank for linux or specify a device, example: /dev/ttyACM0
    interface = SerialInterface.SerialInterface(5)

    # Go to initial starting position
    interface.send_absolute_angles(0,10,10,0)

    # INITIALIZING CAMERA
    sys.stdout.write("Initiliazing the Camera..." )

    camera = AR_Camera.Camera(CAMERA_ID, VIDEO_MODE, DUCKY, DUCKYBOT, OBSTACLE)
    camera.start()

    print "OK!\n"

    # DISPLAY CAMERA PROPERTIES
    print camera

    object_selection = '''OBJECTS:
    1. ducky
    2. duckybot
    3. obstacle
    '''

    options = '''COMMANDS:
    reset
    top view
    get data
    touch
    track
    grab ducky
    place ducky
    search
    search and place
    arm calibration
    touch test
    stack
    quit

    ENTER COMMAND:
    '''
    while True:
        print options
        command = raw_input("").lower()

        if command == "reset":
            # Reset angles to default positition
            interface.send_absolute_angles(0,10,10,0)

        elif command == "arm calibration":
	    # THIS METHOD SHOULD BE MODIFIED TO VARIOUS DESIRED POINTS
	    # AFTER EACH RUN, RECORD THE ACCURACY OF THE MOVEMENT
            angles = interface.current_status.angles[0:3]
            # Get current XYZ
            P0t = DobotModel.forward_kinematics(angles)
            # Move down towards the table
            goal = [P0t[0],P0t[1], -20]
            move_xyz(interface, goal)
            # Move 150mm in the y-axis
            goal2 = np.reshape(goal ,(3, 1)) + np.reshape(np.array([0,150,0]) ,(3, 1))
            move_xyz(interface, goal2)


        elif command == "touch test":
	    # Get camera capture data
            data = camera.get_all_poses()
	    # If all objects are in view, perform the touch test
            if data[0] != [None, None] and data[1] != [None, None] and data[2] != [None, None]:
		# Determine desired XYZ coordinates for tag 1 + 2 + 3
                goal_1 = get_xyz(interface, data[0][0])
                goal_2 = get_xyz(interface, data[1][0])
                goal_3 = get_xyz(interface, data[2][0])
		# Touch first tag and then reset
                move_xyz(interface, goal_1, False)
                interface.send_absolute_angles(0,10,10,0)
		# Touch second tag and then reset
                move_xyz(interface, goal_2, False)
                interface.send_absolute_angles(0,10,10,0)
		# Touch third tag and then reset
                move_xyz(interface, goal_3, False)
                interface.send_absolute_angles(0,10,10,0)

        elif command == "stack":
	        # Get camera data
                data = camera.get_all_poses()
		
		# Only stack if all tags are in view
                if data[0] != [None, None] and data[1] != [None, None] and data[2] != [None, None]:
		    # Stack the AR tags
                    stack(interface,data)
		    # Reset
                    interface.send_absolute_angles(0,10,10,0)


        elif command == "top view":
            # Get current XYZ
            P0t = DobotModel.forward_kinematics(interface.current_status.get_angles())
            P0t[2] = 100
            move_xyz(interface, P0t)

        elif command == "get data":
	    # This will print out the current camera capture data
            get_data(camera)

        elif command == "grab ducky":
            # Move 30mm above the ducky
            ducky_xyz = DUCKY_POS + np.array([[0], [0], [30]])
            move_xyz(interface, ducky_xyz, True)
            # Move directly onto the ducky with pump on
            move_xyz(interface, DUCKY_POS, True)
            # Move back up 30mm with the pump still on
            move_xyz(interface, ducky_xyz, True)
            # Move to default starting position
            interface.send_absolute_angles(0,10,10,0, interface.MOVE_MODE_JOINTS, True)

        elif command == "place ducky":
            # Which object to place ducky on?
            print object_selection
            print "Which object to place ducky on?"
            selection = int(input(""))
            # Get AR tag position
            data = camera.get_all_poses()[selection - 1]
            target = data[0]
            if target != None:
                place_ducky(interface,target, 0, False)
            else:
                print "Object is not available."

        elif command == "touch":
            # Which object to touch?
            print object_selection
            print "Which object to touch?"
            selection = int(input(""))
            # Get data
            data = camera.get_all_poses()[selection - 1]
            # Touch the object if it is available
            if data != [None,None]:
                touch(interface, selection - 1, 1)
            else:
                print "Object is not available."

        elif command == "track":
            # Which object to track?
            print object_selection
            selection = int(input("Which object to track: "))
            # Begin Tracking
            track(interface, camera, selection - 1)

	elif command == "search":
	    #Which object to search for?
	    print object_selection
	    print "Which object to search for?"
	    selection = int(input(""))
	    # search until tag is found
	    search(interface, camera, selection - 1)	
	    
        elif command == "search and place":
            # Which object to search for?
            print object_selection
            print "Which object to search for?"
            selection = int(input(""))
	    # Do you wish to activate clean mode?
            print "Clean workspace? (Throw unknown obstacles in trash) (Y/N)"
	    clean_mode = raw_input("")	 
	    if clean_mode.upper() == "Y":
	        clean_mode = True
	    else:
		clean_mode = False	    
	    # Do you wish to activate path planning?
            print "Avoid obstacle? (Use path planning) (Y/N)"
	    path_planning = raw_input("")
	    if path_planning.upper() == "Y":
	        path_planning = True
	    else:
		path_planning = False
            target = None
            # search until tag is found (Loop ensures that search is reactivated if tag is lost)
            while target is None:
		return_code = search(interface, camera, selection - 1, clean_mode, path_planning)
		# User requested to exit the program during the search process
		if return_code == None:
		    break;
                # Get AR tag position
                data = camera.get_all_poses()[selection - 1]
                target = data[0]
                # angle = data[1] # THIS NEEDS TO BE CORRECTED
                if target != None:
                    # Place the ducky on the target
                    place_ducky(interface,target, 0, path_planning) # 0 NEEDS TO BE ANGLE
                    interface.send_absolute_angles(0,10,10,0)
                    break;



        elif command == "quit":
            # Reset to default position
            interface.send_absolute_angles(0,10,10,0)
            # Stop the camera device
            sys.stdout.write("Releasing camera...")
            camera.release()
	    camera.join()
            print "OK!"
            break;
