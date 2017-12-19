#!/usr/bin/env python

import rospy
import math
import csv
import sys
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped
from wifianalyzer_teamgrapes import wifiutils as wu

lastLinVel = 0.0
lastAngVel = 0.0
oldPose = Pose2DStamped()
f = open('./heatmapdata.csv', 'wt')
writer = csv.writer(f)

# integrates the linear and angular velocities to calculate displacement
def integrate(theta_dot, v, dt):
    # change in theta
    theta_delta = theta_dot * dt
    
    # if the angular velocity is very small
    # assume we are moving in a straight line
    if abs(theta_dot) < 0.00001:
        x_delta = v * dt
        y_delta = 0
    else:
        # otherwise we are moving along the arc of a circle
        r = v / theta_dot
        x_delta = r * math.sin(theta_delta)
        y_delta = r * (1.0 - math.cos(theta_delta))
    
    return [theta_delta, x_delta, y_delta]
    
# updates the position based on car velocities
def updatePosition(twist_msg):
    global lastLinVel, lastAngVel, oldPose
    
    if oldPose.header.stamp.to_sec() > 0:
        dt = (twist_msg.header.stamp - oldPose.header.stamp).to_sec()
        [theta_delta, x_delta, y_delta] = integrate(lastAngVel, lastLinVel, dt)
        
        oldPose.x = oldPose.x + x_delta * math.cos(oldPose.theta) - y_delta * math.sin(oldPose.theta)
        oldPose.y = oldPose.y + x_delta * math.sin(oldPose.theta) + y_delta * math.cos(oldPose.theta)
        oldPose.theta = oldPose.theta + theta_delta
        
    oldPose.header.stamp = twist_msg.header.stamp
    lastAngVel = twist_msg.omega
    lastLinVel = twist_msg.v
   # linearVel = twist_msg.v
   # angularVel = twist_msg.omega
   # dt = twist_msg.header.stamp.to_sec() - curTime.to_sec()
   # curTime = twist_msg.header.stamp
    
   # theta = theta + angularVel*dt
   # xPos = math.cos(theta)*linearVel*dt + xPos
   # yPos = math.sin(theta)*linearVel*dt + yPos
    
    strength = wu.get_duckietown_strength_perlin(oldPose.x, oldPose.y)
    
    writer.writerow((oldPose.x, oldPose.y, strength))
    rospy.loginfo(rospy.get_caller_id() + " robot is now at x:%f y:%f theta:%f with strength %d", oldPose.x, oldPose.y, oldPose.theta, strength)

# listens for updates to the lane_pose topic and updates known position accordingly
def listener():
    global writer
    writer.writerow(('x','y','strength'))
    print "listening\n"
    #rospy.init_node('heatmap_data_node', anonymous=True)
    rospy.Subscriber("/teamgrapes/lane_controller_node/car_cmd", Twist2DStamped, updatePosition)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('heatmap_data_node', anonymous=True)
    curTime = rospy.get_rostime()
    print "starting\n"
    listener()
