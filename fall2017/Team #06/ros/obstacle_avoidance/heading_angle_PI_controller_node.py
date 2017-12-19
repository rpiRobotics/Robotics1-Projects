#!/usr/bin/env python

"""
Edited by Ruixuan Yan
Implements PI heading angle controller on top of the lane controller
"""
import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, LanePose, AprilTagDetectionArray, AprilTagDetection

class ar_tag_lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None

        self.pub_counter = 0

        # Setup parameters
        self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        self.sub_ar_tag_detections = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.cbTags, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

        self.stop_msg = self.get_stop_msg()
        self.found_obstacle = False

        self.stop_dist = 0.5
        self.slow_down_dist = 1
        self.current_v = self.v_bar


    def get_stop_msg(self):
        stop_msg = Twist2DStamped()
        stop_msg.v = 0.0
        stop_msg.omega = 0.0
        return stop_msg

    def get_msg(self, v, omega):
        stop_msg = Twist2DStamped()
        stop_msg.v = v
        stop_msg.omega = omega
        return stop_msg      

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        #d_offset = 0.0
	d_offset = 0.5

        self.v_bar = self.setupParameter("~v_bar",v_bar) # Linear velocity
        self.k_d = self.setupParameter("~k_d",k_theta) # P gain for theta
        self.k_theta = self.setupParameter("~k_theta",k_d) # P gain for d
        self.d_thres = self.setupParameter("~d_thres",theta_thres) # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",d_thres) # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset",d_offset) # a configurable offset from the lane position

    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")

        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_old))
            rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres
            self.d_offset = d_offset

    
    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)
        
        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self,car_cmd_msg):

        #wheels_cmd_msg = WheelsCmdStamped()
        #wheels_cmd_msg.header.stamp = stamp
        #speed_gain = 1.0
        #steer_gain = 0.5
        #vel_left = (speed_gain*speed - steer_gain*steering)
        #vel_right = (speed_gain*speed + steer_gain*steering)
        #wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        #wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)

        self.pub_car_cmd.publish(car_cmd_msg)
        #self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def cbPose(self,lane_pose_msg):
        self.lane_reading = lane_pose_msg 
	

        #cross_track_err = lane_pose_msg.d - self.d_offset
        #heading_err = lane_pose_msg.phi
	j = 1
	k1 = 0.5235
	k2 = 1.2678
	heading_err1 = 0
	if self.found_obstacle:
		while j < 10:
			cross_track_err = lane_pose_msg.d - 0.0
			heading_err = lane_pose_msg.phi - 0.3467
	

        		car_control_msg = Twist2DStamped()
        		car_control_msg.header = lane_pose_msg.header
	
	   # added stop flag
        		car_control_msg.v = self.current_v  #*self.speed_gain #Left stick V-axis. Up is positive
			print("V command %f" % car_control_msg.v)
        
        		if math.fabs(cross_track_err) > self.d_thres:
           			cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres
        		car_control_msg.omega =  k1 * heading_err - k1 * heading_err1 + k2 * heading_err #*self.steer_gain #Right stick H-axis. Right is negative
       # if not self.found_obstacle
			self.publishCmd(car_control_msg)
			heading_err1 = heading_err
			j = j + 1  
 
        # controller mapping issueheading_err = lane_pose_msg.phiheading_err = lane_pose_msg.phi
        # car_control_msg.steering = -car_control_msg.steering
        # print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        # self.pub_.publish(car_control_msg)
        else:
        	cross_track_err = lane_pose_msg.d + 0.0
        	heading_err = lane_pose_msg.phi


        	car_control_msg = Twist2DStamped()
        	car_control_msg.header = lane_pose_msg.header

           # added stop flag
     		car_control_msg.v = self.current_v/2  #*self.speed_gain #Left stick V-axi$
		
      		if math.fabs(cross_track_err) > self.d_thres:
            		cross_track_err = cross_track_err / math.fabs(cross_track_err) #* se$
        	car_control_msg.omega =  self.k_d * cross_track_err + self.k_theta * heading_err #* he$
		print("V and omega (%f, %f)" %(car_control_msg.v, car_control_msg.omega))
       		self.publishCmd(car_control_msg)

        rospy.loginfo("ar_lane_lane_control pose callback")
        # debuging
        #self.pub_counter += 1
        #if self.pub_counter % 50 == 0:
        #    self.pub_counter = 1
        #    print "lane_controller publish"
        #    print car_control_msg

    def cbTags(self, tag_msg):
        rospy.loginfo("ar_tag_lane_control ar tag callback")
        self.found_obstacle = False
        self.process_tags(tag_msg)

    def process_tags(self, tag_msg):
	self.current_v = self.v_bar
        for tag_detection in tag_msg.detections:
            tag_id = int(tag_detection.id)
            z_pos = tag_detection.pose.pose.position.z
	    x_pos = tag_detection.pose.pose.position.x
            if z_pos < self.stop_dist:
		#self.publishCmd(self.stop_msg)
                #self.current_v = self.v_bar
		self.found_obstacle = True
                rospy.loginfo("Found z pos to be %f - stopping" %(z_pos))
           # elif z_pos < self.slow_down_dist:
            #    self.current_v = self.v_bar / 2
             #   rospy.loginfo("Found z pos to be %f - slowing down" %(z_pos))
           # elif z_pos >= self.slow_down_dist:
            #    self.current_v = self.v_bar
             #   rospy.loginfo("Found z pos to be %f - speeding up" %(z_pos))


if __name__ == "__main__":
    rospy.init_node("ar_tag_lane_controller",anonymous=False)
    lane_control_node = ar_tag_lane_controller()
    rospy.spin()