// Declarations
#include "proto1_controller.h"

// Error threshold
#define THRESHOLD 0.002

// Constructor
Proto1Controller::Proto1Controller(float p)
{
	// Set propotional gain
	p_ = p;
}

// Control action
void Proto1Controller::control(std::vector<float> reference)
{
	// Position error
	float error = reference[0] - sensor_->getValue();

	// If error is above threshold
	if (error > THRESHOLD || error < - THRESHOLD)
	{
		// Actuate
		actuator_->setValue(p_ * error);
	}
	// If error is under threshold
	else
	{
		// Set 0
		actuator_->setValue(0);
	}

	
}