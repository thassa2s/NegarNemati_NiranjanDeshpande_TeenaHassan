README
------

- Copy the folder raw_arm_cart_traj_control to the folder raw_manipulation in RoboCupAt Work.
- cd to raw_manipulation/raw_arm_cart_traj_control 
- mkdir build
- cd build
- cmake ..
- rosmake raw_arm_cart_traj_control
- If there are errors in build, fix them, and then retry rosmake raw_arm_cart_traj_control
- After build is successful:
-- roslaunch youbot_description youbot_description.launch
-- roslaunch youbot_description youbot_publisher.launch
-- roslaunch hbrs_arm_cart_control arm_cartesian_control_youbot.launch
-- roslaunch raw_arm_cart_traj_control simple_arm_cartesian_trajectory_control_youbot.launch
-- rosrun raw_arm_cart_traj_control simple_arm_ctesian_trajectory_control
-- rostopic pub /simple_arm_cartesian_trajectory_control/cartesian_trajectory_control_goal_posiition geometry_msgs/PoseStamped '{header: {frame_id: /base_link}, pose: {position: {x: 0.040, y: 0.007, z: 0.534}}}' 
