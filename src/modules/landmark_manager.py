# Python 2.7
import os
import rospy
from modules.landmark_checker import load_landmarks, find_nearby_landmarks


class LandmarkManager(object):
    def __init__(self):
        # Resolve landmarks.json relative to this file
        module_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(module_dir, "landmarks.json")

        self.max_distance = rospy.get_param("~landmark_distance", 5.0)

        try:
            self.landmarks = load_landmarks(json_path)
            rospy.loginfo("Loaded %d landmarks", len(self.landmarks))
        except Exception as e:
            rospy.logerr("Failed to load landmarks: %s", e)
            self.landmarks = []

    def get_landmarks_near(self, pose):
        """
        pose: geometry_msgs/Pose
        returns: dict or None
        """
        if pose is None:
            return None

        x = pose.position.x
        y = pose.position.y

        return find_nearby_landmarks(
            (x, y),
            self.landmarks,
            self.max_distance
        )

