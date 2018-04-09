#ifndef VISION_CONTROL_TOOLS_VISION_CONTROLLER_H
#define VISION_CONTROL_TOOLS_VISION_CONTROLLER_H

// ROS
#include "std_msgs/Float32MultiArray.h"					// std_msgs::Float32MultiArray

// Vision control tools
#include "vision_control_tools/ros_object_array.h"		// vision_control_tools::ros_object_array

// Template class defining a visual controller
class VisionController
{
	public:
		// Generates reference from updated objects
		// To be overwritten by children
		virtual std_msgs::Float32MultiArray objectsToReference(const vision_control_tools::ros_object_array::ConstPtr& obj) = 0;
};

#endif // VISION_CONTROL_TOOLS_VISION_CONTROLLER_H