// ROS
#include "ros/ros.h"						// ros::*
#include "std_msgs/Float32MultiArray.h"		// std_msgs::Float32MultiArray

// Main function
int main(int argc, char* argv[])
{
	// Initialize ROS
	ros::init(argc, argv, "vision_contol_reference_server");
	ros::NodeHandle n;

	// Initialize subscriber
	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("reference_update", 1);

	// Spin
	ros::spinOnce();
}