#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Main application node for cs4023major_project
Python 2.7
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from modules.queue_manager import QueueManager
from modules.landmark_manager import LandmarkManager
from modules.nav_interface import NavInterface

class ApplicationCore(object):
    """
    High-level orchestrator node that wires together:
    - queue system
    - landmark detection interface
    - robot navigation interface
    """

    def __init__(self):
        rospy.init_node('cs4023_application_core')

        rospy.loginfo("Starting CS4023 Application Core...")

        # -----------------------
        # Initialize sub-modules
        # -----------------------
        self.queue_manager = QueueManager()
        self.landmarks = LandmarkManager()
        self.nav = NavInterface()

        # -----------------------
        # Subscribers
        # -----------------------
        # Example: subscribe to robot pose (from amcl or gazebo)
        self.pose_sub = rospy.Subscriber(
            "/robot_pose",
            Pose,
            self.cb_robot_pose,
            queue_size=10
        )

        # Example: subscribe to external commands (from web UI, etc.)
        self.cmd_sub = rospy.Subscriber(
            "/tourbot/user_commands",
            String,
            self.cb_user_command
        )

        # -----------------------
        # Publishers
        # -----------------------
        self.eta_pub = rospy.Publisher(
            "/tourbot/queue_eta",
            String,
            queue_size=10
        )

        self.landmark_pub = rospy.Publisher(
            "/tourbot/nearby_landmarks",
            String,
            queue_size=10
        )

        rospy.loginfo("CS4023 Application Core initialized.")

    # -----------------------
    # Callbacks
    # -----------------------
    def cb_robot_pose(self, msg):
        """Receives robot location; triggers landmark scan + nav integration"""
        # Landmark detection
        lm = self.landmarks.get_landmarks_near(msg)

        if lm:
            self.landmark_pub.publish(str(lm))

        # Queue system might also need robot pose later (for ETA)
        self.queue_manager.update_robot_pose(msg)

    def cb_user_command(self, msg):
        """Handles external commands such as:
           'QUEUE_ME'
           'STATUS'
           'CANCEL'
        """
        cmd = msg.data.upper().strip()
        rospy.loginfo("Received command: %s", cmd)

        if cmd == "QUEUE_ME":
            eta, location = self.queue_manager.add_person()
            out = "ETA: %s minutes | Wait at: %s" % (eta, location)
            self.eta_pub.publish(out)
            rospy.loginfo(out)

        elif cmd == "STATUS":
            out = self.queue_manager.status()
            self.eta_pub.publish(out)

        elif cmd == "CANCEL":
            out = self.queue_manager.cancel()
            self.eta_pub.publish(out)

        else:
            rospy.logwarn("Unknown command received: %s", cmd)

    # -----------------------
    # Main Loop
    # -----------------------
    def spin(self):
        rospy.loginfo("Application Core running...")
        rospy.spin()


if __name__ == "__main__":
    app = ApplicationCore()
    app.spin()

