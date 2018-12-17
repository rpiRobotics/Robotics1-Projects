#!/usr/bin/env python
# Commands the duckie to go stop and gives a turn at a stop line

import rospy
from std_msgs.msg import String, Int16
from duckietown_msgs.msg import BoolStamped, Twist2DStamped
import copy

# Runs when the stop line is triggered
# Copied from go_around
def control_car(data):

    global pub_turn_type, counter

    # Only run when we are at a stop line
    if data.data == True:
    
    	# Array of the t,v,w commands
#    	turns = [ [3, 0, 0] ]

    	# Create a list of the t,v,w commands to go around
#    	manuever = list()
#    	for turn in turns:
#    	    manuever.append((turn[0],Twist2DStamped(v=turn[1],omega=turn[2])))

    	# Run each manuever (from open_loop_control_intersection_node code)
#    	for index, pair in enumerate(manuever):
#            cmd = copy.deepcopy(pair[1])
#	    start_time = rospy.Time.now()
#	    end_time = start_time + rospy.Duration.from_sec(pair[0])
#	    while rospy.Time.now() < end_time:
#	        cmd.header.stamp = rospy.Time.now()
#	        pub_cmd.publish(cmd)
	
	# First publish a stop for 2 seconds
	pub_turn_type.publish(-1)
	rospy.sleep(2)

   	# Tell the open loop intersection control the turn
      	point = counter%4
        # If we're at the second turn (0-3), its a right
        if point == 1:
            turn_type = 2
        # Otherwise its straight
        else:
             turn_type = 1
        
        pub_turn_type.publish(turn_type)
	counter +=1
def start():

    # Initialize the node
    rospy.init_node('stopper_node')

    # Set up the publisher
    global pub_turn_type
    pub_turn_type = rospy.Publisher('open_loop_intersection_control_node/turn_type', Int16, queue_size=1)

    # Subscribe to lane_blocked
    rospy.Subscriber('stop_line_filter_node/at_stop_line', BoolStamped, control_car)
 
    # Setup the counter that will keep track of what turn to take
    global counter
    counter = 0
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
	start()
    except rospy.ROSInterruptException:
	pass
