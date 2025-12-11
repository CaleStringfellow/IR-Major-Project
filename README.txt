(do the chmod)
chmod +x ~/catkin_ws/src/cs4023major_project/src/test_app.sh
chmod +x ~/catkin_ws/src/cs4023major_project/src/cs4023major_app.py

(rebuild the workspace as usual)

(start the main node)
rosrun cs4023major_project cs4023major_app.py

(publish a command to join the queue)
rostopic pub /tourbot/user_commands std_msgs/String "data: 'QUEUE_ME'" -1

(check queue status)
rostopic pub /tourbot/user_commands std_msgs/String "data: 'STATUS'" -1

(cancel the first person)
rostopic pub /tourbot/user_commands std_msgs/String "data: 'CANCEL'" -1

(PLACEHOLDER) (publish a test pose)
rostopic pub /robot_pose geometry_msgs/Pose "{position: {x:1.0, y:2.0, z:0.0}, orientation: {x:0.0, y:0.0, z:0.0, w:1.0}}" -1

I have a script in there as well but couldn't figure out how to make it work fully.
