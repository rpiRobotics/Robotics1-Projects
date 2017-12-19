#!/usr/bin/env python

import rospy
import numpy as np
from duckietown_msgs.msg import LanePose
import csv

# tile length in centimeters
TILE_LENGTH = 60.0

# width across both lanes in centimeters
TOTAL_LANE_WIDTH = 48.0

class PoseEstimator:
    def __init__(self, outputFile):
        f = open(outputFile, 'wt')
        self.writer = csv.writer(f)
        self.oldLP = LanePose()
        self.z = 0.0
        self.tiles_crossed = 0
        rospy.Subscriber('/teamgrapes/lane_filter_node/lane_pose', LanePose, self.estimatePose)

    def estimatePose(self, lanepose_msg):
        if self.oldLP.header.stamp.to_sec() > 0:
            dt = (lanepose_msg.header.stamp - self.oldLP.header.stamp).to_sec()
            
            if (self.oldLP.phi < 0.0001):
                delta_z = dt
            else:
                v = self.oldLP.d / np.sin(self.oldLP.phi)
                dz = np.sqrt(v**2 - self.oldLP.d**2)
                delta_z = dz * dt
                
            self.z += delta_z

        self.oldLP = lanepose_msg
        rospy.loginfo('lane pose is d = %f phi = %f current z is: %f', self.oldLP.d, self.oldLP.phi, self.z)
        if (self.z / 2.5 > (self.tiles_crossed+1)):
            rospy.loginfo('tile %d has been crossed', self.tiles_crossed)
            self.tiles_crossed += 1

if __name__ == '__main__':
    rospy.init_node('lane_pose_estimator_node', anonymous=True)
    pe = PoseEstimator('./estimatedpose.csv')
    rospy.spin()
