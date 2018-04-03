// Declarations
#include "vision_control/example_controller.h"

// Constructor
// Initialize controller parameters here
ExampleController::ExampleController(int P, std::string object_name, std::string robot_name)
{
	// Initialize proportional gain
	P_ = P;

	// Intiialize names to be tracked
	object_name_ = object_name;
	robot_name_ = robot_name;
}

// Generates reference from updated objects
vision_control_tools::reference ExampleController::objectsToReference(const vision_control_tools::ros_object_array::ConstPtr& obj)
{
	// Create reference message
	vision_control_tools::reference ref;

	/* Example: Moving robot to object in the Z direction with standard P action */

	// Set invalid ID's for object and robot
	int object_id = -1;
	int robot_id = -1;

	// Loop over updated objects to find object and robot
	for (int i = 0; i < obj->objects.size(); i++)
	{
		// If object in array has same name as desired object
		if (obj->objects[i].name == object_name_)
		{
			// Set object ID
			object_id = i;
		}

		// If object in array has same name as robot
		if (obj->objects[i].name == robot_name_)
		{
			// Set robot ID
			robot_id = i;
		}
	}

	// If both the desired object and robot are found
	if (object_id != -1 && robot_id != -1)
	{
		// Create reference
		ref.reference.push_back(P_ * (obj->objects[object_id].z - obj->objects[robot_id].z));
	}
	else
	{
		// Set reference to 0
		ref.reference.push_back(0);
	}

	/* Example end */

	// Return reference message
	return ref;
}
