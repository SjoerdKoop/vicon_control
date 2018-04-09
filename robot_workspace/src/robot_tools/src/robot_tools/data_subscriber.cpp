// ROS
#include "ros/ros.h"							// ros::*

// Vision control tools
#include "robot_tools/data_update.h"			// robot_tools::data_update
#include "robot_tools/data_update_array.h"		// robot_tools::data_update_array

// Handle for when an update message has arrived
void onDataUpdate(const robot_tools::data_update_array::ConstPtr& msg)
{
	// Print notification
	std::cout << "### Data update received ###" << std::endl;

	// For each data update
	for (int i = 0; i < msg->updates.size(); i++)
	{
		// Print data update
		std::cout << "Name: " << msg->updates[i].name;
		std::cout << "\tValue: " <<msg->updates[i].value;
		std::cout << std::endl;
	}
}

// Main function
int main(int argc, char* argv[])
{
	// Initialize ROS
	ros::init(argc, argv, "robot_tools_data_subscriber");
	ros::NodeHandle n;

	// Initialize subscriber
	ros::Subscriber sub = n.subscribe<robot_tools::data_update_array>("data_update", 1, onDataUpdate);

	// Spin
	ros::spin();
}