#!/usr/bin/env python
#Goal of this script:

#Use camera feedback to control motion of duckiebot in conjunction with move2_node?

#Inputs:
#d = distance from lane center line
#phi = bearing with respect to straight
#z = distance from AR Tag?

#Outputs:
#desired position to be sent to move2 node
#OR
#desired velocities to be sent to inverse_kinematics node

#Tag in Detection array tag.pose.pose.pos.z or something like that follow msg chain
#ONLY WRITTEN FOR ONE TAG
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, AprilTagDetectionArray
import numpy
from geometry_msgs.msg import PoseStamped

class PoseControlNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		
		self.kd = 25
		self.kphi = 6

		self.currentv = .3
		self.currentomega = 0
		self.prevz = 100000

		self.pub_vel = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
		self.sub_lane = rospy.Subscriber("~lane_pose", LanePose, self.lane_callback, queue_size=1)
		self.sub_tag = rospy.Subscriber("~apriltags_in", AprilTagDetectionArray, self.tag_callback, queue_size=1)

		rospy.loginfo("[%s] Initilized", self.node_name)

	def lane_callback(self, lane_pose):
		d = lane_pose.d
		phi = lane_pose.phi 
		kd = self.kd
		kphi = self.kphi
		
		if self.currentv == 0:
			omega_cmd = 0
		else:
			omega_cmd = -1*kphi*phi - kd*(d + .01)

		lane_msg = Twist2DStamped()
		lane_msg.v = self.currentv
		lane_msg.omega = omega_cmd
		self.pub_vel.publish(lane_msg)

	def tag_callback(self, msg):
		for tag in msg.detections:
			tag_position = tag.pose.pose.position
			
			newz = tag_position.z

			if newz < .2 and newz > -.2:
				v_cmd = 0
			elif newz < self.prevz:
				v_cmd = .2
			else:
				v_cmd = -.2

		self.currentv = v_cmd
			
if __name__ == '__main__':
        rospy.init_node('position_controller_node', anonymous=False)
        position_controller_node = PoseControlNode()
        rospy.spin()


