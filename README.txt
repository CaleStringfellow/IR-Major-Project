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

----------------------------
(OPTIONAL OpenAI Q&A FEATURE)
----------------------------

(Q&A lets users send: questions and get a response on /tourbot/qa_response)

(If you DO NOT have an OpenAI API key, SKIP to "End of optional Q&A instructions" below.
 NOTE: If you skip the key setup, the Q&A feature will not function, but the rest of the app (queue/go_to/etc.) still works.)

(If you DO have an OpenAI API key)
(install dependency for the OpenAI client - Python 2 / ROS Melodic)
sudo apt-get update
sudo apt-get install -y python-requests

(Local key file - still NOT committed)
cd ~/catkin_ws/src/cs4023major_project
cat > set_openai_key.sh << 'EOF'
export OPENAI_API_KEY="PASTE_YOUR_KEY_HERE"
EOF
chmod 600 set_openai_key.sh
echo "set_openai_key.sh" >> .gitignore

(run this BEFORE starting the main node)
source ./set_openai_key.sh

----------------------------
(run the Q&A viewer)
----------------------------

(do this chmod one time)
chmod +x ~/catkin_ws/src/cs4023major_project/scripts/qa_echo.py

(run in a separate terminal to see Q&A output)
rosrun cs4023major_project qa_echo.py

----------------------------
(Q&A question examples + format)
----------------------------

(send questions from another terminal)
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'ASK how hard is robotics engineering?'"
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'ASK whats the square root of pi?'"
rostopic pub -1 /tourbot/user_commands std_msgs/String "data: 'ASK how many people are in the queue?'"

----------------------------
End of optional Q&A instructions
----------------------------

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

