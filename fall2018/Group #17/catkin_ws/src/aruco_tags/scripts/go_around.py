#!/usr/bin/env python
# Commands the duckie to go around an obstacle

import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped, Twist2DStamped
import copy

# Runs when the lane is blocked
def control_car(data):

    global pub_lane_blocked, pub_cmd

    # Only run when the lane is blocked
    if data.data == True:
    
    	# Array of the t,v,w commands
	# Middle command: [1, .43, 0]
    	turns = [ [1.4, .25, 3], [1.4, .25, -3], [.4, .5, 0], [1, .25, -2], [.4, .25, 2] ]

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

   	# Tell fsm that we are done going around
    	b = BoolStamped()
    	b.header.stamp = rospy.Time.now()
    	b.data = False

    	# Publish to the lane_blocked topic to be read by fsm state
    	pub_lane_blocked.publish(b)

def start():

    # Initialize the node
    rospy.init_node('go_around')

    # Set up the publisher
    global pub_lane_blocked, pub_cmd
    pub_lane_blocked = rospy.Publisher('/howard17/obstacle_safety_node/lane_blocked', BoolStamped, queue_size=1)
    pub_cmd = rospy.Publisher('/howard17/obstacle_safety_node/go_around_cmd', Twist2DStamped, queue_size=1)

    # Subscribe to lane_blocked
    rospy.Subscriber('/howard17/obstacle_safety_node/lane_blocked', BoolStamped, control_car)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
	start()
    except rospy.ROSInterruptException:
	pass
