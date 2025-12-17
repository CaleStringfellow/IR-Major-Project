(do the chmod)
chmod +x ~/catkin_ws/src/cs4023major_project/src/test_app.sh
chmod +x ~/catkin_ws/src/cs4023major_project/src/cs4023major_app.py

(rebuild the workspace as usual)

(start the main node)
rosrun cs4023major_project cs4023major_app.py

(or, the now better version)
(start the world file)
roslaunch cs4023major_project world.launch

(now the test file actually works for the queue, status, and cancel, as well as feeding it some fake odometry if you need to test it moving a little)
rosrun cs4023major_project test_app.sh

(rudimentary "go to" command)
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'GO_TO East Atrium'"

(to have open in seperate cmd when you want to see a cleaner version of the queue "UI", or test odometry and nearby landmarks)
rostopic echo /odom
rostopic echo /tourbot/queue_eta
rostopic echo /tourbot/nearby_landmarks

(publish a command to join the queue)
rostopic pub /tourbot/user_commands std_msgs/String "data: 'QUEUE_ME'" -1

(check queue status)
rostopic pub /tourbot/user_commands std_msgs/String "data: 'STATUS'" -1

(cancel the first person)
rostopic pub /tourbot/user_commands std_msgs/String "data: 'CANCEL'" -1

(publish a test movement)
rostopic pub -1 /odom nav_msgs/Odometry "
pose:
  pose:
    position:
      x: 3.2
      y: -4.9
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"

