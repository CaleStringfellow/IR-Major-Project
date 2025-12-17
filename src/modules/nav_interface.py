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
        elif status == 4:  # ABORTED
            rospy.logwarn("Navigation goal aborted (likely unreachable or blocked).")
        elif status == 5:  # REJECTED
            rospy.logwarn("Navigation goal rejected.")
        elif status == 8:  # PREEMPTED
            rospy.loginfo("Navigation goal preempted.")
        else:
            rospy.logwarn("Navigation goal status: %d", status)

    def send_goal(self, x, y, yaw=0.0):
        """
        Send a navigation goal in the map.
        Args:
            x: X coordinate in map frame
            y: Y coordinate in map frame
            yaw: Orientation angle in radians (default: 0.0)
        """
        import math

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self._goal_reached = False
        self.goal_pub.publish(goal)

        rospy.loginfo("Sent navigation goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)

    def goal_reached(self):
        return self._goal_reached


