#!/usr/bin/env python
# Commands the duckie to turn around when it sees an obstacle blocking road

import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped, Twist2DStamped
import copy

# Runs when the road is blocked
def control_car(data):

    # Only run when the road is blocked
    if data.data == True:
	global pub_road_blocked, pub_cmd
    
    	# Array of the t,v,w commands
    	turns = [ [2, .15, 3] ]

    	# Create a list of the t,v,w commands to go around
    	manuever = list()
    	for turn in turns:
    	    manuever.append((turn[0],Twist2DStamped(v=turn[1],omega=turn[2])))

    	# Run each manuever (from open_loop_control_intersection_node code)
    	for index, pair in enumerate(manuever):
            cmd = copy.deepcopy(pair[1])
	    start_time = rospy.Time.now()
	    end_time = start_time + rospy.Duration.from_sec(pair[0])
	    while rospy.Time.now() < end_time:
	        cmd.header.stamp = rospy.Time.now()
	        pub_cmd.publish(cmd)

   	# Tell fsm that we are done turning around
    	b = BoolStamped()
    	b.header.stamp = rospy.Time.now()
    	b.data = False

    	# Publish to the road_blocked topic to be read by fsm state
    	pub_road_blocked.publish(b)

def  start():

    # Initialize the node
    rospy.init_node('turn_around')

    # Set up the publisher
    global pub_road_blocked, pub_cmd
    pub_road_blocked = rospy.Publisher('obstacle_safety_node/road_blocked', BoolStamped, queue_size=1)
    pub_cmd = rospy.Publisher('obstacle_safety_node/turn_around_cmd', Twist2DStamped, queue_size=1)
    
    # Subscribe to road_blocked
    rospy.Subscriber('obstacle_safety_node/road_blocked', BoolStamped, control_car)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
	start()
    except rospy.ROSInterruptException:
	pass
