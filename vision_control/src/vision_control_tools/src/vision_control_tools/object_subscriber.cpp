// ROS
#include "ros/ros.h"									// ros::*

// Vision control tools
#include "vision_control_tools/ros_object.h"			// vision_control_tools::ros_object
#include "vision_control_tools/ros_object_array.h"		// vision_control_tools::ros_object_array

// Handle for when an update message has arrived
void onObjectUpdate(const vision_control_tools::ros_object_array::ConstPtr& msg)
{
	// Print notification
	std::cout << "### Object update received ###" << std::endl;

	// For each object
	for (int i = 0; i < msg->objects.size(); i++)
	{
		// Print data
		std::cout << "Object: " << msg->objects[i].name;
		std::cout << "\tX: " <<msg->objects[i].x;
		std::cout << "\tY: " <<msg->objects[i].y;
		std::cout << "\tZ: " <<msg->objects[i].z;
		std::cout << "\tRX: " <<msg->objects[i].rx;
		std::cout << "\tRY: " <<msg->objects[i].ry;
		std::cout << "\tRZ: " <<msg->objects[i].rz;
		std::cout << std::endl;
	}
}

// Main function
int main(int argc, char* argv[])
{
	// Initialize ROS
	ros::init(argc, argv, "vision_contol_object_subscriber");
	ros::NodeHandle n;

	// Initialize subscriber
	ros::Subscriber sub = n.subscribe<vision_control_tools::ros_object_array>("object_update", 1, onObjectUpdate);

	// Spin
	ros::spin();
}