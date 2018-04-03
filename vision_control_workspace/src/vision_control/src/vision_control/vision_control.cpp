// ROS
#include "ros/ros.h"							// ros::*, ROS_INFO

// Vision control tools
#include "vision_control_tools/vision.h"		// Vision::* (always include)

// Vision example
#include "vision_control/example_controller.h"	// ExampleController::* (include your controller)

// Main function
int main(int argc, char* argv[])
{
	// Initialize ROS
	ros::init(argc, argv, "vision_control");

	// Create your controller
	ExampleController* controller = new ExampleController(10, "marker0", "robot");

	// Initialize VisionControl namespace with your controller
	Vision::init(controller);

	// Notify user
	ROS_INFO("Vision controller started!");

	// Invoke spin. This makes the program halt while receiving callbacks from subscribers
	// In this case, the subscriber to object_update in VisionControl
	ros::spin();
	
	// When the program has ended for some reason (ctrl+c, error, ...), terminate namespace for a clean exit
	Vision::terminate();

	// Return standard exit code
	return EXIT_SUCCESS;
}