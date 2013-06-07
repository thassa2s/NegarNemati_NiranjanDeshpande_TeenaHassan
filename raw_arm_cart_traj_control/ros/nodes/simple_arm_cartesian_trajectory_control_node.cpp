/*
 * ros_simple_arm_cartesian_trajectory_control.cpp
 *
 *      Created on: June 06, 2013
 *      Author: negar, niranjan, teena
 */


#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

geometry_msgs::Point goalPosition;

std::string root_name = "DEFAULT_CHAIN_ROOT";
std::string tooltip_name = "DEFAULT_CHAIN_TIP";

tf::TransformListener *tf_listener;

bool active = false;
ros::Time t_last_command;

void getCurrChainTipCartesianPosition(geometry_msgs::Point& currPosition) {
        //chain tip wrt root
        if ( !tf_listener ) return;
        try {
	    tf::StampedTransform stampedTransform;

            std::cout << "Finding description of frame " << tooltip_name.c_str() << " relative to frame " << root_name.c_str() << std::endl;

            tf_listener->lookupTransform(root_name, tooltip_name, ros::Time(), stampedTransform);

            tf::Vector3 origin = stampedTransform.getOrigin();
	    currPosition.x = origin.getX();
            currPosition.y = origin.getY();
            currPosition.z = origin.getZ();

	} catch(...) {
            std::cout << "Could not get current chain tip position in Cartesian space:" << std::endl; }
         
}

void cartTrajGoalCallback(geometry_msgs::PoseStampedConstPtr trajGoalPosition) {
        std::cout << "Trajectory goal position received. Frame: " << trajGoalPosition->header.frame_id << " x: " << trajGoalPosition->pose.position.x << " y: " << trajGoalPosition->pose.position.y << " z: " << trajGoalPosition->pose.position.z << std::endl;
	
        if ( !tf_listener ) return;
	try {
	    geometry_msgs::Vector3Stamped position_in;
	    geometry_msgs::Vector3Stamped position_out;
	    position_in.header = trajGoalPosition->header;
	    position_in.vector.x = trajGoalPosition->pose.position.x;
	    position_in.vector.y = trajGoalPosition->pose.position.y;
	    position_in.vector.z = trajGoalPosition->pose.position.z;

	    tf_listener->transformVector(root_name, position_in, position_out);

            geometry_msgs::PointStamped position1_in;
            geometry_msgs::PointStamped position1_out;
            position1_in.header = trajGoalPosition->header;
            position1_in.point  = trajGoalPosition->pose.position;
	    tf_listener->transformPoint(root_name, position1_in, position1_out);
	    std::cout << "Alternate method: Transformed goal position: root_name: " << root_name.c_str() << " x: " << position1_out.point.x << " y: " << position1_out.point.y << " z: " << position1_out.point.z << std::endl; 


	    goalPosition.x = position_out.vector.x;
	    goalPosition.y = position_out.vector.y;
	    goalPosition.z = position_out.vector.z;

            t_last_command = ros::Time::now();
	} catch(...) {
	  ROS_ERROR("Could not transform frames %s -> %s", trajGoalPosition->header.frame_id.c_str(), root_name.c_str());}
	std::cout << "Transformed goal position: root_name: " << root_name.c_str() << " x: " << goalPosition.x << " y: " << goalPosition.y << " z: " << goalPosition.z << std::endl; 

	//What is the current pose wrt root?
        geometry_msgs::Point currPosition;
        getCurrChainTipCartesianPosition(currPosition);

        std::cout << "Current chain tip position in Cartesian space is: x: " << currPosition.x <<" y: " << currPosition.y << " z: " << currPosition.z << std::endl;

	active = true;
}

/*bool watchdog() {

	double watchdog_time = 0.3;
	if (active==false) {
		return false;
	}

	ros::Time now = ros::Time::now();

	ros::Duration time = (now - t_last_command);

	if ( time > ros::Duration(watchdog_time) ) {
		active = false;
		stopMotion();
		return false;
	}

	return true;
}*/

int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_arm_cartesian_trajectory_control");
	ros::NodeHandle node_handle("~");
	tf_listener = new tf::TransformListener();

        double rate = 50; //50Hz
	
	double default_speed = 0.01;
	
	int goal_pose_x = 1;
	int goal_pose_y = 1;
	int goal_pose_z = 1;
	
	//topic to publish to
	std::string cart_vel_topic = "cartesian_velocity_command"; //change: read from param
	
	//topic to subscribe to: a new topic cartesian_trajectory_control_goal_posiition //two arguments: frame_id and the position coordinates PoseStamped.
	std::string cart_traj_goal_pos_topic =  "cartesian_trajectory_control_goal_posiition"; //change: read from param
	
         if (!node_handle.getParam("/raw_manipulation/simple_arm_cartesian_trajectory_control/root_name", root_name)) {
		ROS_ERROR("No parameter for root_name specified");
		return -1;
        }

        if (!node_handle.getParam("/raw_manipulation/simple_arm_cartesian_trajectory_control/tip_name", tooltip_name)) {
		ROS_ERROR("No parameter for tip_name specified");
		return -1;
        }


        //register subscriber
	ros::Subscriber sub_cart_traj_goal = node_handle.subscribe(cart_traj_goal_pos_topic,
			1, cartTrajGoalCallback);
			//loop with 50Hz

	ros::Rate loop_rate(rate);

	while (ros::ok()) {

		ros::spinOnce();

		/*if(watchdog()) {

		}*/

		loop_rate.sleep();
	}	
}
