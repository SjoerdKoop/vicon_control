#ifndef VISION_CONTROL_TOOLS_VISION_H
#define VISION_CONTROL_TOOLS_VISION_H

// ROS
#include "ros/ros.h"									// ros::*

// Vision control tools
#include "vision_control_tools/reference.h"				// vision_control_tools::reference
#include "vision_control_tools/ros_object_array.h"		// vision_control_tools::ros_object_array
#include "vision_control_tools/vision_controller.h"		// VisionController

// Visual namespace
namespace Vision
{
	ros::Publisher pub_;			// Publisher to reference_update
	ros::Subscriber sub_;			// Subscriber to object_update
	VisionController* controller_;	// Active vision controller

	// Initializes the namespace
	void init(VisionController* controller);

	// Handle for when an update message has arrived
	void onObjectUpdate(const vision_control_tools::ros_object_array::ConstPtr& msg);

	// Terminates the namespace
	void terminate();

}; // VisualControl namespace

#endif // VISION_CONTROL_TOOLS_VISION_H