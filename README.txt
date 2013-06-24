README
------

- Inside the raw_manipulation directory of RoboCupAtWork repository, execute roscreate-pkg raw_arm_cart_traj_control std_msgs rospy roscpp hbrs_arm_cart_control
- Copy the contents of the folder raw_arm_cart_traj_control to the newly created package.
- cd to raw_arm_cart_traj_control 
- mkdir build
- cd build
- cmake ..
- rosmake raw_arm_cart_traj_control
- If there are errors in build, fix them, and then retry rosmake raw_arm_cart_traj_control
- After build is successful:
-- roslaunch youbot_description youbot_description.launch
-- roslaunch youbot_description youbot_publisher.launch
-- roslaunch raw_arm_cart_traj_control simple_arm_cartesian_trajectory_control_youbot.launch
-- rostopic pub /raw_manipulation/simple_arm_cartesian_trajectory_control/cartesian_trajectory_control_goal_position geometry_msgs/PoseArray '{header: {frame_id: /arm_link_0}, poses: [{position: {x: 0.2, y: 0.08, z: 0.086}}, {position: {x: 0.3, y: 0, z: 0.086}}, {position: {x: 0.3, y: 0.08, z: 0.086}}]}'


