#ifndef VISION_CONTROL_TOOLS_VISION_CONTROLLER_H
#define VISION_CONTROL_TOOLS_VISION_CONTROLLER_H

// Vision control tools
#include "vision_control_tools/reference.h"				// vision_control_tools::reference
#include "vision_control_tools/ros_object_array.h"		// vision_control_tools::ros_object_array

// Template class defining a visual controller
class VisionController
{
	public:
		// Generates reference from updated objects
		// To be overwritten by children
		virtual vision_control_tools::reference objectsToReference(const vision_control_tools::ros_object_array::ConstPtr& obj) = 0;
};

#endif // VISION_CONTROL_TOOLS_VISION_CONTROLLER_H