// Declarations
#include "proto1_controller.h"

// Error threshold
#define THRESHOLD 0.001

// Constructor
Proto1Controller::Proto1Controller(float p)
{
	// Set propotional gain
	p_ = p;
}

// Control action
void Proto1Controller::control(std::vector<float> reference)
{
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
}