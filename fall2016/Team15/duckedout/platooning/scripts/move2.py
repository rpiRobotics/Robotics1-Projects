#!/usr/bin/env python
#plan is to input desired pose, generate trajectory to pose, drive along trajectory


#generating a Twist2DStamped message and sending it to inverse_kinematcs_node will drive the wheels
#to generate a Twist2DStamped message we need to determine v and omega in car frame


#we also need to know the current pose of the bot and the desired pose
#current pose can be kept track of by forward_kinematics_node & velocity_to_pose_node
#desired pose will be set beforehand

#velocity should be discretized since the duckiebot runs off of processing individual messages
#two options, track the pose output by velocity_to_pose_node and control velocity input using error based control
#simply feed in a string of velocities and assume that the bot will move correctly


#PLAN

#READ DESIRED X Y THETA

#DETERMINE COEFFICENTS OF SPLINE MOVE TO POSE X Y THETA FROM X0 Y0 THETA0

#DISCRETIZE LAMBDA AND DETERMINE SERIES OF POSES

#DETERMINE V AND OMEGA TO MOVE TO EACH POINT LINEARLY

#FEED SEQUENTIALLY INTO INVERSE NODE


#possible issues, how long to wait in between messages?

import rospy
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped
import numpy

class Move2Node(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		
#		self.movetime = 1
		self.v_max = .2
		self.omega_max = 4
		self.pub_vel = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1) 
		self.sub = rospy.Subscriber("~move_cmd", Pose2DStamped, self.move_callback, queue_size=1) 
		
		rospy.loginfo("[%s] Initilized.", self.node_name)

	def move_callback(self, move_cmd):
		deltax = move_cmd.x
		deltay = move_cmd.y
		deltatheta = move_cmd.theta
		hyp = numpy.sqrt(numpy.square(deltax)+numpy.square(deltay))
		
		if hyp == 0:
			deltatheta1 = 0
			pass
		else:
			deltatheta1 = numpy.arccos(deltax / hyp )
			self.movetime1 = numpy.absolute(deltatheta1 / self.omega_max)

			if deltatheta1 == 0:
				omega_cmd_1 = 0
			elif deltay < 0:
				deltatheta1 = -1*deltatheta1
				omega_cmd_1 = deltatheta1 / self.movetime1
			else:
				omega_cmd_1 = deltatheta1 / self.movetime1
		
			twist1_msg = Twist2DStamped()
			twist1_msg.v = 0
			twist1_msg.omega = omega_cmd_1
			self.pub_vel.publish(twist1_msg)
			rospy.sleep(self.movetime1)
			
			self.movetime2 = numpy.absolute(hyp / self.v_max)
			v_cmd_1 = hyp / self.movetime2
			v1_msg = Twist2DStamped()
			v1_msg.v = v_cmd_1
			v1_msg.omega = 0
			self.pub_vel.publish(v1_msg)
			rospy.sleep(self.movetime2)

		deltatheta2 = deltatheta - deltatheta1
		if deltatheta2 == 0:
			omega_cmd_2 = 0
			pass
		else:
			self.movetime3 = numpy.absolute(deltatheta2 / self.omega_max)
			omega_cmd_2 = deltatheta2 / self.movetime3 
		twist2_msg = Twist2DStamped()
		twist2_msg.v = 0
		twist2_msg.omega = omega_cmd_2
		self.pub_vel.publish(twist2_msg)
		rospy.sleep(self.movetime3) 

#		if omega_cmd < 0.000001:
#			v_cmd = deltax / self.movetime
#		else:
#			v_cmd = (deltay * omega_cmd) / (numpy.cos(deltatheta)
		
#		if (v_cmd>self.v_max):
#			self.movetime = v_cmd/self.v_max
#			v_cmd = self.v_max
#		elif (omega_cmd>self.omega_max):
#			self.movetime = omega_cmd/self.omega_max
#			omega_cmd = self.omega_max
#		else:
#			pass
		
#		msg_car_cmd = Twist2DStamped()
#		msg_car_cmd.v = v_cmd
#		msg_car_cmd.omega = omega_cmd
#		self.pub_vel.publish(msg_car_cmd)
#		rospy.sleep(self.movetime)
#		self.movetime = 1

		#Testing with individual commands requires an end command
		end_move_cmd = Twist2DStamped()
		end_move_cmd.v = 0
		end_move_cmd.omega = 0
		self.pub_vel.publish(end_move_cmd) 

if __name__ == '__main__':
	rospy.init_node('move2_node', anonymous=False)
	move2_node = Move2Node()
	rospy.spin()
