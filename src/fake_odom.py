#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Fake odometry publisher for testing without physical robot
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from std_msgs.msg import Header
import math

class FakeOdomPublisher:
    def __init__(self):
        rospy.init_node('fake_odom_publisher', anonymous=True)
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Fake odometry publisher started")
        
    def run(self):
        seq = 0
        while not rospy.is_shutdown():
            # Create odometry message
            odom = Odometry()
            odom.header.seq = seq
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            
            # Set pose
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
            
            # Set covariance (identity-like)
            odom.pose.covariance[0] = 0.1  # x
            odom.pose.covariance[7] = 0.1  # y
            odom.pose.covariance[35] = 0.1  # yaw
            
            # Set twist (no movement for now)
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0
            
            self.odom_pub.publish(odom)
            
            seq += 1
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = FakeOdomPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass


