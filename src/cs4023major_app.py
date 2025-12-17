#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Main application node for cs4023major_project
Python 2.7
"""

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
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
	self.pose_sub = rospy.Subscriber(
	    "/odom",
	    Odometry,
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
        """Receives robot odometry; extracts pose"""
        pose = msg.pose.pose

        lm = self.landmarks.get_landmarks_near(pose)
        if lm:
            self.landmark_pub.publish(str(lm))

        # Store the Pose object, not the Odometry message
        self.queue_manager.update_robot_pose(pose)

    def cb_user_command(self, msg):
        """Handles external commands such as:
           QUEUE_ME
           STATUS
           CANCEL
        """
        cmd = msg.data.upper().strip()
        rospy.loginfo("Received command: %s", cmd)

        if cmd == "QUEUE_ME":
            nearest = None

            if self.queue_manager.robot_pose is not None:
                try:
                    nearest = self.landmarks.get_landmarks_near(
                        self.queue_manager.robot_pose
                    )
                except AttributeError:
                    # Handle case where robot_pose might be wrong type
                    rospy.logwarn("Robot pose not in correct format, skipping landmark detection")
                    nearest = None

            eta, location = self.queue_manager.add_person(nearest)

            out = "ETA: %d minutes | Wait at: %s" % (eta, location)
            self.eta_pub.publish(out)
            rospy.loginfo(out)

        elif cmd == "STATUS":
            out = self.queue_manager.status()
            self.eta_pub.publish(out)
            rospy.loginfo(out)

        elif cmd == "CANCEL":
            out = self.queue_manager.cancel()
            self.eta_pub.publish(out)
            rospy.loginfo(out)
        elif cmd.startswith("GO_TO"):
            # Example: GO_TO East Atrium
            target_name = cmd.replace("GO_TO", "").strip()
        
            for lm in self.landmarks.landmarks:
                if lm["name"].upper() == target_name.upper():
                    self.nav.send_goal(lm["x"], lm["y"])
                    out = "Navigating to %s" % lm["name"]
                    self.eta_pub.publish(out)
                    rospy.loginfo(out)
                    return

            self.eta_pub.publish("Landmark not found.")

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


