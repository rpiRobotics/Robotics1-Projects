#!/usr/bin/env python3
import morality.srv
import duckietown_msgs
import rospy
import sys

def judge(duck_list):
	rospy.wait_for_service('judgement/judge')
	judge_svc = rospy.ServiceProxy('judgement/judge', morality.srv.judge)
	ducks = duckietown_msgs.msg.ObstacleProjectedDetectionList()
	for d in duck_list:
		duck = duckietown_msgs.msg.ObstacleProjectedDetection()
		duck.location.x = d
		duck.location.y = 20
		duck.type.type = duckietown_msgs.msg.ObstacleType.DUCKIE
		duck.distance = 3.14
		ducks.list.append(duck)
	
	return judge_svc(0, ducks)

try:
	print(judge([float(arg) for arg in sys.argv[1:]]))
except rospy.ServiceException as e:
	print("Failed to call service: ", e)
