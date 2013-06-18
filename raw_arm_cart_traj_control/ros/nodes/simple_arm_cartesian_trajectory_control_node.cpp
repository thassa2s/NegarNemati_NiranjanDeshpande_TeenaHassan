/*
 * ros_simple_arm_cartesian_trajectory_control.cpp
 *
 *      Created on: June 06, 2013
 *      Author: negar, niranjan, teena
 */


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

geometry_msgs::Point goalPosition;
geometry_msgs::Point currPosition;
geometry_msgs::TwistStamped cartesianVelocityMsg;

std::string root_name = "DEFAULT_CHAIN_ROOT";
std::string tooltip_name = "DEFAULT_CHAIN_TIP";

tf::TransformListener *tf_listener;

bool goalReceived = false;
bool goalReached = true;
ros::Time t_last_command;

ros::Publisher cartesianVelocityPublisher;

geometry_msgs::Point lineSegmentStart;
geometry_msgs::Vector3 directionVector;

bool atSingularity = false;
bool jointInterpolationModeOn = false;

double distanceToGoal = 0.0;
double goalTolerance = 0.005;
double driftTolerance = 0.0005;

void compute3DLineParameters()
{
  lineSegmentStart=currPosition;
  directionVector.x=goalPosition.x-lineSegmentStart.x;
   directionVector.y=goalPosition.y-lineSegmentStart.y;
    directionVector.z=goalPosition.z-lineSegmentStart.z;
}

double computeDistanceFrom3DLine(geometry_msgs::Point p)
{
  double directionVectorLengthSq=directionVector.x*directionVector.x+directionVector.y*directionVector.y+directionVector.z*directionVector.z;
  double start2pVectorLengthSq;
  double dotProduct;
  double squaredDistance;
  geometry_msgs::Vector3 start2pVector ;
  start2pVector.x=lineSegmentStart.x-p.x;
  start2pVector.y=lineSegmentStart.y-p.y;
  start2pVector.z=lineSegmentStart.z-p.z;
  start2pVectorLengthSq=start2pVector.x*start2pVector.x+start2pVector.y*start2pVector.y+start2pVector.z*start2pVector.z;
  dotProduct=directionVector.x*start2pVector.x+directionVector.y*start2pVector.y+directionVector.z*start2pVector.z;
  squaredDistance=((start2pVectorLengthSq*directionVectorLengthSq)-(dotProduct*dotProduct))/directionVectorLengthSq;
  return sqrt(squaredDistance);
}
  
bool hasDriftedFromLine()
{
  double distance;
  distance=computeDistanceFrom3DLine(currPosition);
  std::cout << "Distance from line: " << distance << std::endl;
  if (distance > driftTolerance)
    return true;
  return false;
}

void updateDistanceToGoal()
{
  geometry_msgs::Vector3 displacement;
  displacement.x = goalPosition.x-currPosition.x;
  displacement.y = goalPosition.y-currPosition.y;
  displacement.z = goalPosition.z-currPosition.z;
  distanceToGoal=sqrt(displacement.x * displacement.x + displacement.y * displacement.y + displacement.z * displacement.z);
  std::cout << "Distance to goal: " << distanceToGoal << std::endl;
}

void setCartesianVelocityToZero()
{
            cartesianVelocityMsg.header.frame_id = root_name.c_str();
            cartesianVelocityMsg.twist.linear.x = cartesianVelocityMsg.twist.linear.y = cartesianVelocityMsg.twist.linear.z = 0;
            cartesianVelocityMsg.twist.angular.x = cartesianVelocityMsg.twist.angular.y = cartesianVelocityMsg.twist.angular.z = 0;

}

void computeVelocityVector() {
            //std::strcpy(cartesianVelocityMsg.header.frame_id, root_name);
            double normalizationConstant = 1.0;
            cartesianVelocityMsg.header.frame_id = root_name.c_str();
            cartesianVelocityMsg.twist.linear.x = (goalPosition.x - currPosition.x);
            cartesianVelocityMsg.twist.linear.y = (goalPosition.y - currPosition.y);    
            cartesianVelocityMsg.twist.linear.z = (goalPosition.z - currPosition.z);
            std::cout << "Cartesian velocity vector: frame_id: " << cartesianVelocityMsg.header.frame_id << std::endl; 
            cartesianVelocityMsg.twist.angular.x = cartesianVelocityMsg.twist.angular.y = cartesianVelocityMsg.twist.angular.z = 0;
            std::cout << "Cartesian velocity vector: linear vel: " << cartesianVelocityMsg.twist.linear.x << " " << cartesianVelocityMsg.twist.linear.y << " " << cartesianVelocityMsg.twist.linear.z << std::endl;
            normalizationConstant = cartesianVelocityMsg.twist.linear.x * cartesianVelocityMsg.twist.linear.x;
            normalizationConstant += cartesianVelocityMsg.twist.linear.y * cartesianVelocityMsg.twist.linear.y;
            normalizationConstant += cartesianVelocityMsg.twist.linear.z * cartesianVelocityMsg.twist.linear.z ;
            normalizationConstant = sqrt( normalizationConstant );
            cartesianVelocityMsg.twist.linear.x /= normalizationConstant;
            cartesianVelocityMsg.twist.linear.y /= normalizationConstant;
            cartesianVelocityMsg.twist.linear.z /= normalizationConstant;
            std::cout << "Normalized Cartesian velocity vector: linear vel: " << cartesianVelocityMsg.twist.linear.x << " " << cartesianVelocityMsg.twist.linear.y << " " << cartesianVelocityMsg.twist.linear.z << std::endl;
            std::cout << "Cartesian velocity vector: angular vel: " << cartesianVelocityMsg.twist.angular.x << " " << cartesianVelocityMsg.twist.angular.y << " " << cartesianVelocityMsg.twist.angular.z << std::endl;
            //cartesianVelocityMsg.header.frame_id = "/base_link";
            //cartesianVelocityMsg.twist.linear.x = 0;
            //cartesianVelocityMsg.twist.linear.y = 0;    
            //cartesianVelocityMsg.twist.linear.z = 0.01;
}

void getCurrChainTipCartesianPosition() {
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
            
            std::cout << "Current position: " << currPosition.x <<", " << currPosition.y << ", " << currPosition.z << std::endl;
            std::cout << "Goal position: " << goalPosition.x <<", " << goalPosition.y << ", " << goalPosition.z << std::endl;
	} catch(...) {
            std::cout << "Could not get current chain tip position in Cartesian space:" << std::endl; }
         
}

void cartTrajGoalCallback(geometry_msgs::PoseStampedConstPtr trajGoalPosition) {
        std::cout << "Trajectory goal position received. Frame: " << trajGoalPosition->header.frame_id << " x: " << trajGoalPosition->pose.position.x << " y: " << trajGoalPosition->pose.position.y << " z: " << trajGoalPosition->pose.position.z << std::endl;
	
        if ( !tf_listener ) return;
	try {

            geometry_msgs::PointStamped position_in;
            geometry_msgs::PointStamped position_out;
            position_in.header = trajGoalPosition->header;
            position_in.point  = trajGoalPosition->pose.position;
	    tf_listener->transformPoint(root_name, position_in, position_out);
	    
            std::cout << "Alternate method: Transformed goal position: root_name: " << root_name.c_str() << " x: " << position_out.point.x << " y: " << position_out.point.y << " z: " << position_out.point.z << std::endl; 
	    
            goalPosition = position_out.point;
	    //goalPosition.x = position_out.vector.x;
	    //goalPosition.y = position_out.vector.y;
	    //goalPosition.z = position_out.vector.z;

            t_last_command = ros::Time::now();
	} catch(...) {
	  ROS_ERROR("Could not transform frames %s -> %s", trajGoalPosition->header.frame_id.c_str(), root_name.c_str());}
	std::cout << "Transformed goal position: root_name: " << root_name.c_str() << " x: " << goalPosition.x << " y: " << goalPosition.y << " z: " << goalPosition.z << std::endl; 

	//What is the current pose wrt root?
        getCurrChainTipCartesianPosition();

        std::cout << "Current chain tip position in Cartesian space is: x: " << currPosition.x <<" y: " << currPosition.y << " z: " << currPosition.z << std::endl;
        compute3DLineParameters(); //Line from currPosition to goalPosition
        computeVelocityVector();   

	goalReceived = true;
        goalReached = false;
}

void singularityNotificationCallback(std_msgs::Bool singularityNotificationMsg) {
        std::cout << "Singularity notification received. " << std::endl;
        setCartesianVelocityToZero();
        atSingularity = singularityNotificationMsg.data;
        jointInterpolationModeOn = true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_arm_cartesian_trajectory_control");
	ros::NodeHandle nodeHandle("~");
	tf_listener = new tf::TransformListener();

        double rate = 50; //50Hz
	
	double default_speed = 0.01;
	
	//topic to publish to
	std::string cart_vel_topic = "cartesian_velocity_command"; 
	
	//topic to subscribe to: a new topic cartesian_trajectory_control_goal_position //two arguments: frame_id and the position coordinates PoseStamped.
	std::string cart_traj_goal_pos_topic =  "cartesian_trajectory_control_goal_position"; 
        std::string singularity_notification_topic = "singularity_notification";
	
         if (!nodeHandle.getParam("/raw_manipulation/simple_arm_cartesian_trajectory_control/root_name", root_name)) {
		ROS_ERROR("No parameter for root_name specified");
		return -1;
        }

        if (!nodeHandle.getParam("/raw_manipulation/simple_arm_cartesian_trajectory_control/tip_name", tooltip_name)) {
		ROS_ERROR("No parameter for tip_name specified");
		return -1;
        }

        //publisher registration
        cartesianVelocityPublisher=nodeHandle.advertise<geometry_msgs::TwistStamped>(cart_vel_topic, 1);
     
        //subscriber registration
	ros::Subscriber cart_traj_goal_subscriber = nodeHandle.subscribe(cart_traj_goal_pos_topic,
			1, cartTrajGoalCallback);

        //subscriber registration
	ros::Subscriber singularity_notification_subscriber= nodeHandle.subscribe(singularity_notification_topic,
			1, singularityNotificationCallback);

	//loop with 50Hz

	ros::Rate loop_rate(rate);

	while (ros::ok()) {

		ros::spinOnce();

                //Check if near the goal; if so, reduce the velocity
                //Check if we drifted from the line
                //recompute the drift
                
		/*if(watchdog()) {

		}*/

                if (goalReceived && !goalReached)
	        { 
                     getCurrChainTipCartesianPosition(); 
                     updateDistanceToGoal();
                     if ( atSingularity && jointInterpolationModeOn )
                     {
                          //drive for 2cm through joint interpolation (joint position control)
                          
                     }  
                     else {
                     if ( distanceToGoal > goalTolerance )
                     {
                         if ( hasDriftedFromLine() )
                         {
                             compute3DLineParameters();
                             computeVelocityVector();  
                             std::cout << "Drifted too much from line. Recomputed trajectory and Cartesian velocities..." << std::endl;    
                         }
                         //publish cartesian velocity
                         cartesianVelocityPublisher.publish(cartesianVelocityMsg);     
                     }
                     else
		     {
                         goalReached = true;
                         goalReceived = false;
                         //stop motion
                         setCartesianVelocityToZero();
                         cartesianVelocityPublisher.publish(cartesianVelocityMsg);  
                         std::cout << "Goal Reached... Stopping motion..." << std::endl;    
                     }
                     std::cout << "Cartesian velocity vector: linear vel: " << cartesianVelocityMsg.twist.linear.x << " " << cartesianVelocityMsg.twist.linear.y << " " << cartesianVelocityMsg.twist.linear.z << std::endl;
                     //check if we are near the goal position; if yes, slow down                         
                  }

                }
		loop_rate.sleep();
	}	
}	
