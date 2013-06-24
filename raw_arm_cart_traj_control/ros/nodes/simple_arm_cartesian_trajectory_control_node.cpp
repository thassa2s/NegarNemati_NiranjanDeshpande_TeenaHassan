/*
 * ros_simple_arm_cartesian_trajectory_control.cpp
 *
 *      Created on: June 06, 2013
 *      Author: negar, niranjan, teena
 */

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#define MAX_SIZE_GOAL_LIST 10

geometry_msgs::Point goalPositionList[MAX_SIZE_GOAL_LIST];
geometry_msgs::Point currPosition;
geometry_msgs::TwistStamped cartesianVelocityMsg;

unsigned int numberOfGoalPositions = 0;
unsigned int currentGoalIndex = 0;
unsigned int maxNumberOfGoalPositions = MAX_SIZE_GOAL_LIST;

std::string root_name = "DEFAULT_CHAIN_ROOT";
std::string tooltip_name = "DEFAULT_CHAIN_TIP";

tf::TransformListener *tf_listener;

geometry_msgs::Point lineSegmentStart;
geometry_msgs::Vector3 directionVector;

double distanceToGoal = 0.0;
double goalTolerance = 0.003;
double driftTolerance = 0.0005;

bool atSingularity = false;
bool jointInterpolationModeOn = false;
bool goalReceived = false;
bool goalReached = true;

ros::Publisher cartesianVelocityPublisher;
ros::Time t_last_command;

void compute3DLineParameters()
{
  lineSegmentStart=currPosition;
  directionVector.x=goalPositionList[currentGoalIndex].x-lineSegmentStart.x;
  directionVector.y=goalPositionList[currentGoalIndex].y-lineSegmentStart.y;
  directionVector.z=goalPositionList[currentGoalIndex].z-lineSegmentStart.z;
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
  return sqrt( squaredDistance );
}
  
bool hasDriftedFromLine()
{
  double distance;
  distance=computeDistanceFrom3DLine( currPosition );
  std::cout << "Distance from line: " << distance << std::endl;
  if ( distance > driftTolerance )
    return true;
  return false;
}

void updateDistanceToGoal()
{
  geometry_msgs::Vector3 displacement;
  displacement.x = goalPositionList[currentGoalIndex].x-currPosition.x;
  displacement.y = goalPositionList[currentGoalIndex].y-currPosition.y;
  displacement.z = goalPositionList[currentGoalIndex].z-currPosition.z;
  distanceToGoal=sqrt(displacement.x * displacement.x + displacement.y * displacement.y + displacement.z * displacement.z);
  std::cout << "Distance to goal: " << distanceToGoal << std::endl;
}

void setCartesianVelocityToZero()
{
  cartesianVelocityMsg.header.frame_id = root_name.c_str();
  cartesianVelocityMsg.twist.linear.x = cartesianVelocityMsg.twist.linear.y = cartesianVelocityMsg.twist.linear.z = 0;
  cartesianVelocityMsg.twist.angular.x = cartesianVelocityMsg.twist.angular.y = cartesianVelocityMsg.twist.angular.z = 0;
}

void normalize3DVector( geometry_msgs:: Vector3 &vector )
{
  double normalizationConstant = 1.0;
  normalizationConstant = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z ;
  normalizationConstant = sqrt( normalizationConstant );
  vector.x /= normalizationConstant;
  vector.y /= normalizationConstant;
  vector.z /= normalizationConstant;
}

void computeVelocityVector() 
{
  cartesianVelocityMsg.header.frame_id = root_name.c_str();
  cartesianVelocityMsg.twist.linear.x = (goalPositionList[currentGoalIndex].x - currPosition.x);
  cartesianVelocityMsg.twist.linear.y = (goalPositionList[currentGoalIndex].y - currPosition.y);    
  cartesianVelocityMsg.twist.linear.z = (goalPositionList[currentGoalIndex].z - currPosition.z);
  cartesianVelocityMsg.twist.angular.x = cartesianVelocityMsg.twist.angular.y = cartesianVelocityMsg.twist.angular.z = 0;
  
  std::cout << "Cartesian velocity vector: frame_id: " << cartesianVelocityMsg.header.frame_id << std::endl; 
  std::cout << "Cartesian velocity vector: linear vel: " << cartesianVelocityMsg.twist.linear.x << " " << cartesianVelocityMsg.twist.linear.y << " " << cartesianVelocityMsg.twist.linear.z << std::endl;
  
  normalize3DVector( cartesianVelocityMsg.twist.linear );
  
  std::cout << "Normalized vector: linear vel: " << cartesianVelocityMsg.twist.linear.x << " " << cartesianVelocityMsg.twist.linear.y << " " << cartesianVelocityMsg.twist.linear.z << std::endl;
  std::cout << "Cartesian velocity vector: angular vel: " << cartesianVelocityMsg.twist.angular.x << " " << cartesianVelocityMsg.twist.angular.y << " " << cartesianVelocityMsg.twist.angular.z << std::endl;
}

void getCurrChainTipCartesianPosition() 
{
  if ( !tf_listener ) 
    return;
  try 
  {
    tf::StampedTransform stampedTransform;
    std::cout << "Finding description of frame " << tooltip_name.c_str() << " relative to frame " << root_name.c_str() << std::endl;
    tf_listener->lookupTransform(root_name, tooltip_name, ros::Time(), stampedTransform);
    tf::Vector3 origin = stampedTransform.getOrigin();
    currPosition.x = origin.getX();
    currPosition.y = origin.getY();
    currPosition.z = origin.getZ();
    std::cout << "Current position: " << currPosition.x <<", " << currPosition.y << ", " << currPosition.z << std::endl;
  } catch(...) 
    {  ROS_ERROR( "Could not get current chain tip position in Cartesian space:" );  }         
}

void cartTrajGoalCallback( geometry_msgs::PoseArray trajGoalPositionArray ) 
{
  unsigned int i;
  std::cout << "List of trajectory goal positions received. Frame: " << trajGoalPositionArray.header.frame_id << std::endl;
  numberOfGoalPositions = trajGoalPositionArray.poses.size();
  std::cout << "Number of poses: " << numberOfGoalPositions << std::endl;
  if ( numberOfGoalPositions > maxNumberOfGoalPositions ) 
  {   
    std::cout << "Only the first " << maxNumberOfGoalPositions << " goal positions will be processed." << std::endl;
    numberOfGoalPositions = maxNumberOfGoalPositions;
  }
  for ( i = 0; i < numberOfGoalPositions; i++ ) 
  {
    std::cout << " x: " << trajGoalPositionArray.poses[i].position.x << " y: " << trajGoalPositionArray.poses[i].position.y << " z: " << trajGoalPositionArray.poses[i].position.z << std::endl;
    if ( !tf_listener ) 
      return;
    try 
    {
      geometry_msgs::PointStamped position_in;
      geometry_msgs::PointStamped position_out;
      position_in.header = trajGoalPositionArray.header;
      position_in.point  = trajGoalPositionArray.poses[i].position;
      tf_listener->transformPoint(root_name, position_in, position_out);
      goalPositionList[i] = position_out.point;
      std::cout << "Transformed goal position: root_name: " << root_name.c_str() << " x: " << goalPositionList[i].x << " y: " << goalPositionList[i].y << " z: " << goalPositionList[i].z << std::endl; 
      t_last_command = ros::Time::now();
    } catch(...) 
      {  ROS_ERROR( "Frame transformation %s to %s failed.", trajGoalPositionArray.header.frame_id.c_str(), root_name.c_str() );  }
  }
  if( numberOfGoalPositions > 0 )
  {
    currentGoalIndex = 0;
    getCurrChainTipCartesianPosition();
    compute3DLineParameters();
    computeVelocityVector();   
    goalReceived = true;
    goalReached = false;
  }
}

void singularityNotificationCallback( std_msgs::Bool singularityNotificationMsg ) 
{
  std::cout << "Singularity notification received. " << std::endl;
  setCartesianVelocityToZero();
  atSingularity = singularityNotificationMsg.data;
  jointInterpolationModeOn = true;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "simple_arm_cartesian_trajectory_control");
  ros::NodeHandle nodeHandle("~");
  tf_listener = new tf::TransformListener();
  double rate = 50; //50Hz
  ros::Rate loop_rate(rate);

  if (!nodeHandle.getParam("/raw_manipulation/simple_arm_cartesian_trajectory_control/root_name", root_name)) 
  {
    ROS_ERROR("Parameter root_name not specified");
    return -1;
  }
  if (!nodeHandle.getParam("/raw_manipulation/simple_arm_cartesian_trajectory_control/tip_name", tooltip_name)) 
  {
    ROS_ERROR("No parameter for tip_name specified");
    return -1;
  }

  //topic to publish to
  std::string cart_vel_topic = "cartesian_velocity_command"; 
  //topics to subscribe to
  std::string cart_traj_goal_pos_topic =  "cartesian_trajectory_control_goal_position"; 
  std::string singularity_notification_topic = "singularity_notification";
	
  //publisher registration
  cartesianVelocityPublisher=nodeHandle.advertise<geometry_msgs::TwistStamped>(cart_vel_topic, 1);
  //subscriber registration
  ros::Subscriber cart_traj_goal_subscriber = nodeHandle.subscribe(cart_traj_goal_pos_topic, 1, cartTrajGoalCallback);
  //subscriber registration
  ros::Subscriber singularity_notification_subscriber= nodeHandle.subscribe(singularity_notification_topic, 1, singularityNotificationCallback);
  
  while (ros::ok()) 
  {
    ros::spinOnce();
    if (goalReceived && !goalReached)
    { 
      getCurrChainTipCartesianPosition(); 
      updateDistanceToGoal();
      std::cout << "Goal position: " << goalPositionList[currentGoalIndex].x <<", " << goalPositionList[currentGoalIndex].y << ", " << goalPositionList[currentGoalIndex].z << std::endl;
      if ( atSingularity && jointInterpolationModeOn )
      {
        //drive for 2cm through joint interpolation (joint position control)
      }  
      else 
      {
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
          setCartesianVelocityToZero();
          cartesianVelocityPublisher.publish(cartesianVelocityMsg);  
          std::cout << "Goal Reached... Stopping motion..." << std::endl;    
        }
        std::cout << "Cartesian vel vector: linear vel: " << cartesianVelocityMsg.twist.linear.x << " " << cartesianVelocityMsg.twist.linear.y << " " << cartesianVelocityMsg.twist.linear.z << std::endl;
        std::cout << "Goal index: " << currentGoalIndex << ". No. of goals: " << numberOfGoalPositions << std::endl;
        //TODO: check if we are near the goal position; if yes, slow down?                         
      }
    }
    else if (goalReceived && goalReached)
    {
      goalReached = false;
      currentGoalIndex++;
      if ( currentGoalIndex == numberOfGoalPositions )
        goalReceived = false;
      else
      {  
        getCurrChainTipCartesianPosition(); 
        compute3DLineParameters(); //Line from currPosition to goalPosition
        computeVelocityVector();    
        cartesianVelocityPublisher.publish(cartesianVelocityMsg);  
      }
    }
    loop_rate.sleep();
  }	
}	
