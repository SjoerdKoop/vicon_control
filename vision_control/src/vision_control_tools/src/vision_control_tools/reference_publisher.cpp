// ROS
#include "ros/ros.h"						// ros::*
#include "std_msgs/Float32MultiArray.h"		// std_msgs::Float32MultiArray

// System
#include <string>							// std::stof

// Main function
int main(int argc, char* argv[])
{
	// Initialize ROS
	ros::init(argc, argv, "vision_contol_reference_server");
	ros::NodeHandle n;

	// Initialize subscriber
	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("reference_update", 1);

	// If arguments are given
	if (argc > 1)
	{
		// Create reference message from arguments;
		std_msgs::Float32MultiArray ref;			// Reference message

		// For each argument
		for (int i = 1; i < argc; i++)
		{
			// Add argument
			ref.data.push_back(std::stof(argv[i]));
		}

		// Wait for subscriber
		std::cout << "Waiting for a subscriber...";
		while(ros::ok() && pub.getNumSubscribers() == 0)
		{
			usleep(1000);
		}
		std::cout << "connected!" <<std::endl;

		while(ros::ok())
		{
			// Publishe reference
			pub.publish(ref);

			// Wait for 0.01 seconds
			usleep(100000);
		}
	}
	// If there are no arguments
	else
	{
		ROS_INFO("Please specify reference variables");
	}

}