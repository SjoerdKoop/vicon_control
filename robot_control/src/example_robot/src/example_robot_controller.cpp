// Declarations
#include "example_robot_controller.h"

// Error threshold
#define THRESHOLD 0.0001

// Constructor
ExampleRobotController::ExampleRobotController(float p)
{
	// Set propotional gain
	p_ = p;
}

// Control action
void ExampleRobotController::control(std::vector<float> reference)
{
	/* Example: Move actuator to setpoint with standard P action */
	/* Setpoint assumed to be the first variable (reference[0]) */

	float ref = reference[0];				// Position reference
	float pos = sensor_->getValue();		// Position
	float error = ref - pos;				// Position error

	// If error is considerably big
	if (error > THRESHOLD || error < - THRESHOLD)
	{
		actuator_->setValue(p_ * error);
	}
	// If error is within thresholds
	else
	{
		// Set 0
		actuator_->setValue(0);
	}

	/* Example end */
}