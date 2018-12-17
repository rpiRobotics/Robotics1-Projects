#!/usr/bin/env python
# Uses a timer to alert us to when something hasn't been pubished in awhile

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from duckietown_msgs.msg import BoolStamped


# Runs when tag 1 is detected (tag 1 = obstacle blocking lane)
def pose(data):
    
    global min_dist_lane, time_out, flag, pub

    dis = data.position.z
    if dis > min_dist_lane:
        rospy.loginfo('Tag 1 is still far away enough: %s m', dis)
    else:
        rospy.loginfo('Tag 1 is too close: %s m', dis)
	time_out = 0
	flag = True
	b = BoolStamped()
        b.data = True
        pub.publish(b)

# Runs when tag 2 is detected (tag 2 = obstacle blocking road)
def pose2(data):

    global min_dist_road, time_out, flag, pub

    dis = data.position.z

    if dis > min_dist_road:
        rospy.loginfo('Tag 2 is still far away enough: %s m', dis)
    else:
        rospy.loginfo('Tag 2 is too close: %s m', dis)
        time_out = 0
	flag = True
	b = BoolStamped()
	b.data = True
	pub.publish(b)

def timer():

    global time_out, flag, pub

    b = BoolStamped()
    b.data = False
    
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
	    pub.publish(b)
	    time_out = 0
	    flag = False
	    rospy.loginfo('Timed out!')
		
def listener():
    # Initialize the node
    rospy.init_node('aruco_listener')

    global min_dist_lane
    min_dist_lane = .2
    global min_dist_road
    min_dist_road = .3

    # Keeps track of how many seconds have gone by since
    # we last heard a published message
    global time_out
    time_out = 0

    # Keeps track of if we even need to be tracking it
    global flag
    flag = True

    # Publish to the too close topic
    global pub
    pub = rospy.Publisher('/howard17/obstacle_avoid/object_too_close', BoolStamped, queue_size=10)

    # Subscriber to the poses of the tags
    rospy.Subscriber('/aruco_double/pose', Pose, pose)
    rospy.Subscriber('/aruco_double/pose2', Pose, pose2)
    
    # Starting timing things
    timer()
    
    rospy.spin()

if __name__ == '__main__':
    listener()
