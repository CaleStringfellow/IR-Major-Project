#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Main application node for cs4023major_project
Python 2.7
"""

import rospy
import threading
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from modules.queue_manager import QueueManager
from modules.landmark_manager import LandmarkManager
from modules.nav_interface import NavInterface
from modules.openai_qa_client import OpenAIQAClient

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

        self.qa_pub = rospy.Publisher(
            "/tourbot/qa_response",
            String,
            queue_size=10
        )

        self.qa = OpenAIQAClient(model=rospy.get_param("~qa_model", "gpt-5-mini"))

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

        # Queue system might also need robot pose later (for ETA)
        self.queue_manager.update_robot_pose(pose)

    def _safe_text(self, s):
        try:
            if isinstance(s, unicode):
                return s.encode('utf-8')
            return str(s)
        except Exception:
            return "[Q&A output encoding error]"

    def _build_qa_context(self):
        parts = []

        # Queue info (real data from your code)
        parts.append(self.queue_manager.status())

        # Landmarks list (what the robot knows)
        try:
            names = [lm.get("name", "") for lm in self.landmarks.landmarks]
            names = [n for n in names if n]
            if names:
                parts.append("Known landmarks: " + ", ".join(names))
        except Exception:
            pass

        # Nearby landmarks (if pose exists)
        try:
            if self.queue_manager.robot_pose is not None:
                near = self.landmarks.get_landmarks_near(self.queue_manager.robot_pose)
                if near:
                    parts.append("Nearby landmarks: " + str(near))
        except Exception:
            pass

        return "\n".join(parts)

    def _answer_question(self, question):
        try:
            instructions = (
                "You are a short Q&A assistant for a TurtleBot2 ROS Melodic Gazebo demo. "
                "Use the provided CONTEXT when it is relevant (queue/landmarks/capabilities). "
                "If the question is general, answer from general knowledge. "
                "Answer in 1-3 sentences. "
                "Keep it short, but finish your last sentence (end with . ! or ?). "
                "Do not use emojis or special symbols. Use plain text only."
            )

            context = self._build_qa_context()
            prompt = "CONTEXT:\n%s\n\nQUESTION:\n%s" % (context, question)

            answer = self.qa.ask(prompt, instructions=instructions, max_output_tokens=500)
        except Exception as e:
            answer = "Q&A disabled/error: %s" % str(e)
        self.qa_pub.publish(self._safe_text(answer))


    def cb_user_command(self, msg):
        """Handles external commands such as:
           QUEUE_ME
           STATUS
           CANCEL
        """
        raw = msg.data.strip()
        cmd = raw.upper().strip()
        rospy.loginfo("Received command: %s", cmd)

        if cmd.startswith("ASK "):
            question = raw[4:].strip()

            if not question:
                self.qa_pub.publish("Usage: ASK <your question>")
                return

            self.qa_pub.publish("TourBot: thinking...")

            t = threading.Thread(target=self._answer_question, args=(question,))
            t.daemon = True
            t.start()
            return

        if cmd == "QUEUE_ME":
            nearest = None

            if self.queue_manager.robot_pose is not None:
                nearest = self.landmarks.get_landmarks_near(
                    self.queue_manager.robot_pose
                )

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

