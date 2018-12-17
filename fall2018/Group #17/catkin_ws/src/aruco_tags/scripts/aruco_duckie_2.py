#!/usr/bin/env python
# Simple listener that subscribes to aruco tags

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from duckietown_msgs.msg import BoolStamped

# Runs when tag 1 is detected (tag 1 = obstacle blocking lane)
def pose(data):

    global min_dist_lane, pub_tag_1, active
    
    if active:

        # Get the distance away from the duckiebot
        dis = data.position.z
        rospy.loginfo('Tag 1 (Obstacle blocking lane) detected.') 

        # If its still far away enough, we're okay
        if dis > min_dist_lane:
             rospy.loginfo('Tag 1 is still far away enough: %s m', dis)
    
        # If its too close, publish true to too_close
        else:
	    # Set up a BoolStamped to publish
	    b = BoolStamped()
	    b.data = True

	    # Print a message
	    rospy.loginfo('Tag 1 is too close: %s m', dis)

	    # Publish to the pub_tag_1 topic to be read by fsm state
	    pub_tag_1.publish(b)

# Runs when tag 2 is detected (tag 2 = obstacle blocking road)
def pose2(data):
  
    global min_dist_road, pub_tag_2, active

    if active:
	
        # Get the distance away from the duckiebot
        dis = data.position.z
        rospy.loginfo('Tag 2 (Obstacle blocking road) detected.') 

        # If we aren't too close yet, its okay
        if dis > min_dist_road:
            rospy.loginfo('Tag 2 is still far away enough: %s m', dis)
    
        # If we are too close, publish true to too_close
        else:
	    # Set up a BoolStamped to publish
            b = BoolStamped()
            b.data = True
        
	    # Print a message
	    rospy.loginfo('Tag 2 is too close: %s m', dis)

	    # Publish to the pub_tag_2 topic to be read by fsm state
	    pub_tag_2.publish(b)

def switch(data):
    global active
    active = data.data

def  listener():

    # Initialize the node
    rospy.init_node('aruco_duckie_2')
    
    # Set the distances (can evetually make this ros params?)
    global min_dist_lane
    min_dist_lane = .3
    global min_dist_road
    min_dist_road = .4

    # Set up the publishers
    global pub_tag_2, pub_tag_1
    pub_tag_1 = rospy.Publisher('/howard17/obstacle_safety_node/lane_blocked', BoolStamped, queue_size=1)
    pub_tag_2 = rospy.Publisher('/howard17/obstacle_safety_node/road_blocked', BoolStamped, queue_size=1)

    # Subscribe to the nodes that give poses of tags
    rospy.Subscriber('/aruco_double/pose', Pose, pose)
    rospy.Subscriber('/aruco_double/pose2', Pose, pose2)

    # Subscribe to switch
    rospy.Subscriber("/howard17/obstacle_safety_node/switch", BoolStamped, switch, queue_size=1)
    global active

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
	listener()
    except rospy.ROSInterruptException:
	pass
