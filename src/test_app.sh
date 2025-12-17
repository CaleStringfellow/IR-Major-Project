#!/bin/bash

# Test script for cs4023major_project
# Simulates users joining queue and fake odometry updates

source ~/catkin_ws/devel/setup.bash

echo "Adding 3 users to queue..."
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'QUEUE_ME'"
sleep 1
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'QUEUE_ME'"
sleep 1
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'QUEUE_ME'"
sleep 1

echo "Checking queue status..."
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'STATUS'"
sleep 1

echo "Cancelling first person..."
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'CANCEL'"
sleep 1

echo "Checking queue status again..."
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'STATUS'"
sleep 1

echo "Publishing fake odometry (robot movement)..."

rostopic pub -1 /odom nav_msgs/Odometry "
pose:
  pose:
    position:
      x: 1.0
      y: 2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"

sleep 1

rostopic pub -1 /odom nav_msgs/Odometry "
pose:
  pose:
    position:
      x: 5.0
      y: 3.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"

echo "Test script finished."
echo "Check:"
echo "  /tourbot/queue_eta"
echo "  /tourbot/nearby_landmarks"

