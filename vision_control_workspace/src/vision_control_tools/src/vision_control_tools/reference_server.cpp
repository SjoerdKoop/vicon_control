// ROS
#include "ros/ros.h"									// ros::*

// Vision control tools
#include "vision_control_tools/reference.h"			// vision_control_tools::reference

// Main function
int main(int argc, char* argv[])
{
	// Initialize ROS
	ros::init(argc, argv, "vision_contol_reference_server");
	ros::NodeHandle n;

	// Initialize subscriber
	ros::Publisher pub = n.advertise<vision_control_tools::reference>("reference_update", 1);

	// Spin
	ros::spinOnce();
}