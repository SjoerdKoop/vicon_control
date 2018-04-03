#ifndef VISION_CONTROL_EXAMPLE_CONTROLLER_H
#define VISION_CONTROL_EXAMPLE_CONTROLLER_H

// ROS
#include "std_msgs/Float32MultiArray.h"					// std_msgs::Float32MultiArray

// System
#include <string>										// std::string

// Vision control tools
#include "vision_control_tools/vision_controller.h"		// VisionController

// An example controller
class ExampleController : public VisionController
{
	public:
		// Constructor
		// Initialize controller parameters here
		ExampleController(int P, std::string object_name, std::string robot_name);

		// Generates reference from updated objects
		// Differences from VisionController:
		// Is not virtual (virtual keyword missing before function definition)
		// Is overrrides parent function (override keyword after function definition)
		std_msgs::Float32MultiArray objectsToReference(const vision_control_tools::ros_object_array::ConstPtr& obj) override;

	private:
		std::string object_name_;	// The name of the object
		std::string robot_name_;	// The name of the robot
		double P_;					// Proportional gain
};

#endif // VISION_CONTROL_EXAMPLE_CONTROLLER_H