#!/usr/bin/env python

"""
Edited by Zac Ravichandran
Builds on top of existing controller node to implement navigation and
obstacle avoidance
"""

import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, LanePose, AprilTagDetectionArray, AprilTagDetection, BoolStamped
import iterative_optimization_for_waypoints
import time
import graph_routing

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
        self.sub_at_stop_line = rospy.Subscriber("~at_stop_line", BoolStamped, self.cb_stop_line, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

        self.stop_msg = self.get_stop_msg()
        self.found_obstacle = False

        self.stop_dist = 1
        self.slow_down_dist = 1
        self.current_v = self.v_bar

        # for obstacle avoidance
        self.obstacle_x_position = 0
        self.obstacle_z_position = 0
        self.k_o = 1
        self.d_0 = 0.1
        self.maneuver_time = 10
        self.avoidance_time = 2*self.maneuver_time+1

        self.last_header = None
        self.last_phi = 0
        self.last_time = 0
        self.last_id = 0

        self.at_stop_line = False

        # 0: left
        # 1: right
        # 2: straight
        # 3: stop temporarily
        # 4: stop and wait for instructions
        turn_instructions = []
        turn_instructions, end_edge = graph_routing.get_duckietown_route([0,1,2,3,4], 's', end_command=[4])
        self.turn_instructions = turn_instructions
        self.print_route(self.turn_instructions)
        self.last_stop_line = 0
        self.current_instruction = 0

    def print_route(self, route):
        print("Turns: %s" %('%s, ' % ', '.join(map(str, route))))

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
        rospy.set_param(param_name,value) #Write to parameter server for transparency
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        #v_bar = 0.0 # change for experimentation
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0.0

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

    def get_error(self, xd, yd, w,t,v,phi):
        return np.linalg.norm(np.power(xd-inverse_kin.x_t(w,t,v,phi), 2) + \
                np.power(yd - inverse_kin.y_t(w,t,v,phi), 2))  

    def go_to_position_with_inverse_kin_wt(self, header, xd, yd, phi, time_adjust=1):
        v = 0.5
        wg,tg = inverse_kin.get_wt_from_gradient_descent(xd, yd, 0.5, phi, k=0.1, steps=200)
        wn, tn = inverse_kin.get_wt_from_newtonian_descent(xd, yd, 0.5, phi, k=0.1, steps=200)
        errorg = self.get_error(xd, yd, wg, tg, v, phi)
        errorn = self.get_error(xd, yd, wn, tn, v, phi)

        if errorg < errorn:
            w = wg
            t = tg 
            rospy.loginfo("Using Gradient Solution")
        else:
            w = wn 
            t = tn
            rospy.loginfo("Using Newton Solution")

        print("xd, yd, phi: (%f, %f, %f), w: %f, t %f, t_adj %f" % (xd, yd, phi, w, t, t*time_adjust))
        t_adj = t * time_adjust

        if True:
            print("Error: (x_d, y_d) = (%f, %f), (x,y)=(%f,%f)" \
                %(xd, yd, inverse_kin.x_t(w,t_adj, 0.5, phi), inverse_kin.y_t(w,t_adj,0.5,phi)))

        # Note - coordinates for solution are flipped for x axis
        t1 = time.time()
        #while time.time() - t1 <= t:
        self.make_and_send_control_msg(header, 0.5, w)
        rospy.sleep(np.abs(t_adj))
        self.publishCmd(self.get_stop_msg())

        return w, t_adj

    def correct_wv(self, w, v):
        if v < 0:
            rospy.loginfo("Corrected (w,v): (%f, %f)" %(w, v))
            return -1*w, -1*v 
        else:
            return w, v

    def go_to_position_with_inverse_kin_wv(self, header, xd, yd, t, phi, time_adjust=1):
        wg,vg = inverse_kin.get_wv_from_gradient_descent(xd, yd, t, phi, k=0.1, steps=500)
        wg, vg = self.correct_wv(wg, vg)
        wn, vn = inverse_kin.get_wv_from_newtonian_descent(xd, yd, t, phi, k=0.1, steps=500)
        wn, vn = self.correct_wv(wn, vn)
        errorg = self.get_error(xd, yd, wg, t, vg, phi)
        errorn = self.get_error(xd, yd, wn, t, vn, phi)

        if errorg < errorn:
            w = wg
            v = vg 
            rospy.loginfo("Using Gradient Solution")
        else:
            w = wn 
            v = vn
            rospy.loginfo("Using Newton Solution")

        print("xd, yd, phi: (%f, %f, %f), w: %f, v %f, t_adj %f" % (xd, yd, phi, w, v, t*time_adjust))
        t_adj = t * time_adjust

        if True:
            print("Error: (x_d, y_d) = (%f, %f), (x,y)=(%f,%f)" \
                %(xd, yd, inverse_kin.x_t(w, t, v, phi), inverse_kin.y_t(w,t, v, phi)))

        # Note - coordinates for solution are flipped for x axis
        t1 = time.time()
        #while time.time() - t1 <= t:
        self.make_and_send_control_msg(header, v, w)
        rospy.sleep(t_adj)
        self.publishCmd(self.get_stop_msg())

        return w, v

    def make_and_send_control_msg(self, header, v, omega):
        car_control_msg = Twist2DStamped()
        #car_control_msg.header = header 
        car_control_msg.v = v
        car_control_msg.omega = omega  
        print("Sending v: %f, w:%f" %(v, omega))
        self.publishCmd(car_control_msg)

    def turn_left(self):
        left_omega = 1.4
        cmd = self.make_and_send_control_msg(self.last_header, self.v_bar, left_omega)
        rospy.sleep(1.75)

    def turn_right(self):
        right_omega = -2.2
        cmd = self.make_and_send_control_msg(self.last_header, self.v_bar, right_omega)
        rospy.sleep(1)

    def go_straight(self, time):
        straight_omega = 0
        cmd = self.make_and_send_control_msg(self.last_header, self.v_bar, straight_omega)
        rospy.sleep(time)    

    def stop(self, time):
        self.publishCmd(self.get_stop_msg())
        rospy.sleep(time)

    def straight_at_instersection(self):
        t = 2.1
        self.go_straight(t)

    def cb_stop_line(self, msg):
        stop_time_filter = 1.2
        #print("At Stop line")
        if time.time() - self.last_stop_line > stop_time_filter:
            rospy.loginfo(msg.header.stamp)
            self.at_stop_line = True
            self.publishCmd(self.get_stop_msg())
            rospy.sleep(1)

            # if we have more instructions to follow
            if self.current_instruction < len(self.turn_instructions):
                if self.turn_instructions[self.current_instruction] == 0:
                    self.turn_left()
                elif self.turn_instructions[self.current_instruction] == 1:
                    self.turn_right()
                elif self.turn_instructions[self.current_instruction] == 2:
                    self.straight_at_instersection()
                elif self.turn_instructions[self.current_instruction] == 3:
                    self.stop(1)
                elif self.turn_instructions[self.current_instruction] == 4:
                    self.stop(1)
                    # TODO, find other response 
                    self.found_obstacle = True

                self.current_instruction += 1

            # default turn left
            else:
                #self.turn_left()
                self.stop(1)
                self.found_obstacle = True

            rospy.sleep(1)
            self.at_stop_line = False
            self.last_stop_line = time.time()
        else:
            rospy.loginfo("Skipping line at time %f" %(time.time() - self.last_stop_line))
            self.last_stop_line = 0


    def cbPose(self,lane_pose_msg):
        self.current_v = self.v_bar
        self.lane_reading = lane_pose_msg 

        cross_track_err = lane_pose_msg.d - self.d_offset
        heading_err = lane_pose_msg.phi
        self.last_phi = heading_err

        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        self.last_header = lane_pose_msg.header

        car_control_msg.v = self.current_v  #*self.speed_gain #Left stick V-axis. Up is positive
        
        if math.fabs(cross_track_err) > self.d_thres:
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres

        #self.found_obstacle = True
    
        if not self.found_obstacle and not self.at_stop_line:
            #rospy.loginfo("Cross track / phi %f %f, w: %f" %(cross_track_err, heading_err, self.k_d * cross_track_err + self.k_theta * heading_err))
            car_control_msg.omega =  self.k_d * cross_track_err + self.k_theta * heading_err #*self.steer_gain #Right stick H-axis. Right is negative
            self.publishCmd(car_control_msg)

    def cbTags(self, tag_msg):
        #rospy.loginfo("ar_tag_lane_control ar tag callback with %d tags" %(len(tag_msg.detections)))
        self.found_obstacle = False
        self.process_tags(tag_msg)

    def process_tags(self, tag_msg):
    	self.current_v = self.v_bar
        for tag_detection in tag_msg.detections:
            tag_id = int(tag_detection.id)
            z_pos = tag_detection.pose.pose.position.z
            x_pos = tag_detection.pose.pose.position.x 
            y_pos = tag_detection.pose.pose.position.y
            if z_pos < self.stop_dist:
                #self.publishCmd(self.stop_msg)
                self.found_obstacle = True
                #rospy.loginfo("Found z pos to be (x,y,z) = (%f, %f, %f - swerving" %(x_pos, y_pos, z_pos))
                if self.obstacle_z_position  == 0:
                    self.obstacle_z_position = z_pos
                    self.obstacle_x_position = x_pos
                    self.avoidance_time = 0
                    # assume a lag
                    yd = z_pos

                    # left is positive x!
                    xd = self.obstacle_x_position - 0.5
                    yd = self.obstacle_z_position
                    t = 1
                    phi = 0
                    w, v = self.go_to_position_with_inverse_kin_wv(self.last_header, xd, yd, t, phi)

                else:
                    rospy.loginfo("Skipping repeat")
                    self.obstacle_x_position = 0
                    self.obstacle_z_position = 0
                    self.tag_id = -1

if __name__ == "__main__":
    rospy.init_node("ar_tag_lane_controller",anonymous=False)
    lane_control_node = ar_tag_lane_controller()
    rospy.spin()
    