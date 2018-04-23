#ifndef ROBOT_CONTROL_EXAMPLE_ROBOT_CONTROLLER_H
#define ROBOT_CONTROL_EXAMPLE_ROBOT_CONTROLLER_H

// Robot control
#include "robot_controller.h"	// RobotController

// Class defining an example robot controller
class ExampleRobotController : public RobotController
{
	public:
		// Constructor
		ExampleRobotController(float p);
		
		// Control action
		void control(std::vector<float> reference) override;

	private:
		float p_;						// Proportional gain
};

#endif // ROBOT_CONTROL_EXAMPLE_ROBOT_CONTROLLER_H