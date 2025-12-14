#!/usr/bin/env python
# -*- coding: utf-8 -*-
# nav_interface.py

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult


class NavInterface(object):

    def __init__(self):
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal",
            PoseStamped,
            queue_size=1
        )

        self._goal_reached = False

        rospy.Subscriber(
            "/move_base/result",
            MoveBaseActionResult,
            self._result_callback
        )

        rospy.loginfo("NavInterface initialized.")

    def _result_callback(self, msg):
        status = msg.status.status
        if status == 3:  # SUCCEEDED
            self._goal_reached = True
            rospy.loginfo("Navigation goal reached.")

    def send_goal(self, x, y, yaw=0.0):
        """
        Send a navigation goal in the map.
        """

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self._goal_reached = False
        self.goal_pub.publish(goal)

        rospy.loginfo("Sent navigation goal: x=%s, y=%s", x, y)

    def goal_reached(self):
        return self._goal_reached

