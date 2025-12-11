#!/bin/bash

# Test script
# Simulates 3 users joining queue and robot movement

# Make sure ROS is sourced
source ~/catkin_ws/devel/setup.bash

# Step 1: Users queue
echo "Adding 3 users to queue..."
rostopic pub /tourbot/user_commands std_msgs/String "data: 'QUEUE_ME'" -1 &
sleep 1
rostopic pub /tourbot/user_commands std_msgs/String "data: 'QUEUE_ME'" -1 &
sleep 1
rostopic pub /tourbot/user_commands std_msgs/String "data: 'QUEUE_ME'" -1 &
sleep 1

# Step 2: Check status
echo "Checking queue status..."
rostopic pub /tourbot/user_commands std_msgs/String "data: 'STATUS'" -1 &
sleep 1

# Step 3: Cancel first person
echo "Cancelling first person..."
rostopic pub /tourbot/user_commands std_msgs/String "data: 'CANCEL'" -1 &
sleep 1

# Step 4: Simulate robot moving
echo "Sending robot poses..."
rostopic pub /robot_pose geometry_msgs/Pose \
"{position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x:0.0, y:0.0, z:0.0, w:1.0}}" -1 &
sleep 1
rostopic pub /robot_pose geometry_msgs/Pose \
"{position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x:0.0, y:0.0, z:0.0, w:1.0}}" -1 &
sleep 1

echo "Test script finished. Watch the /tourbot/queue_eta and /tourbot/nearby_landmarks topics."

