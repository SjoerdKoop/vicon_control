// Declarations
#include "proto2_controller.h"

// Error threshold
#define THRESHOLD 0.001

// Constructor
Proto2Controller::Proto2Controller(float p)
{
	// Set propotional gain
	p_ = p;
}

// Control action
void Proto2Controller::control(std::vector<float> reference)
{
	float ref = reference[0];				// Position reference
	float pos = sensor_->getValue();		// Position
	float error = ref - pos;				// Position error

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