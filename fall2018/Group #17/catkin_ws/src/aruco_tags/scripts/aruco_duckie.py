#!/usr/bin/env python
# Simple listener that subscribes to aruco tags

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from duckietown_msgs.msg import BoolStamped

# Runs when tag 1 is detected (tag 1 = obstacle blocking lane)
def pose(data):

    global min_dist_lane, time_out, flag, pub_too_close
    
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

	# Publish to the too_close topic:
	pub_too_close.publish(b)

	# Reset the timer and tell timer to start counting
        time_out = 0
        flag = True


# Runs when tag 2 is detected (tag 2 = obstacle blocking road)
def pose2(data):

    global min_dist_road, time_out, flag, pub_too_close

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

	# Publish to the too_close topic
	pub_too_close.publish(b)

	# Reset the timer and tell timer to start counting
	time_out = 0
	flag = True

# Sends a false after the tag has been gone for 5 seconds
def timer():

    global time_out, flag, pub_too_close

    b = BoolStamped()
    b.data = False

    # Essentially a while(true) loop
    while not rospy.is_shutdown():

        # Sleep 1 second
        rospy.sleep(1)

        # If the last message published was true
        if flag == True:

            # Increase the time
            time_out+=1
            rospy.loginfo('Incrementing time out!')

            # If no message has been published in 5 seconds
            # publish "false" to the topic "too close"
        if time_out >= 5:
            pub_too_close.publish(b)
            time_out = 0
            flag = False
            rospy.loginfo('Timed out!')


def listener():

    # Initialize the node
    rospy.init_node('aruco_duckie')
    
    # Set the distances (can evetually make this ros params?)
    global min_dist_lane
    min_dist_lane = .6
    global min_dist_road
    min_dist_road = .6

    # Keeps track of how many seconds have gone by since
    # we last heard a published message
    global time_out
    time_out = 0

    # Keeps track of if we even need to be tracking it
    global flag
    flag = True
    
    
    # Set up the publisher
    # Currently doesn't account for the difference between tags
    global pub_too_close
    pub_too_close = rospy.Publisher('/howard17/obstacle_safety_node/object_too_close', BoolStamped, queue_size=1)
    
    # Subscribe to the nodes that give poses of tags
    rospy.Subscriber('/aruco_double/pose', Pose, pose)
    rospy.Subscriber('/aruco_double/pose2', Pose, pose2)
 
    # Starting timing things
    timer()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
	listener()
    except rospy.ROSInterruptException:
	pass
