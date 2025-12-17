#!/usr/bin/env python
import rospy
from std_msgs.msg import String

_last_msg_time = None

def cb(msg):
    global _last_msg_time
    _last_msg_time = rospy.get_time()
    print("\nTourBot: " + msg.data)

def main():
    global _last_msg_time
    rospy.init_node("qa_echo", anonymous=True)

    print("Q&A is running.")
    print("Listening on /tourbot/qa_response")
    print("Send a question like:")
    print("  rostopic pub -1 /tourbot/user_commands std_msgs/String \"data: 'ASK tell me something about robotics'\"")
    print("Waiting for TourBot replies...")

    rospy.Subscriber("/tourbot/qa_response", String, cb)
    _last_msg_time = rospy.get_time()

    rate = rospy.Rate(1)  # 1 Hz heartbeat
    while not rospy.is_shutdown():
        now = rospy.get_time()
        # If nothing has arrived for 5 seconds, print a small heartbeat
        if now - _last_msg_time >= 5.0:
            print("(still listening...)")
            _last_msg_time = now
        rate.sleep()

if __name__ == "__main__":
    main()
