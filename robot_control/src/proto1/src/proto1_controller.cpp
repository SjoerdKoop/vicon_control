// Declarations
#include "proto1_controller.h"

#include <iostream>

// Error threshold
#define THRESHOLD 0.0001

#define MU 0.05
#define FRICTION 3.4
#define TORQUE_CONSTANT 0.0292		// [Nm/A]
#define STARTING_TORQUE 2 			// [Nm]
#define MASS_BLOCK 2				// [kg]
#define GRAVITY_CONSTANT 9.81		// [m/s^2]

// Constructor
Proto1Controller::Proto1Controller(float p, float d)
{
	// Set propotional gain
	p_ = p;

	// Set derivative gain
	d_ = d;

	// Set last position to 0
	last_pos_ = 0;

	// Set gravitational torque
	g_torque_ = MU * MASS_BLOCK * GRAVITY_CONSTANT;
}

// Control action
void Proto1Controller::control(std::vector<float> reference)
{
	struct timespec current_time;			// Current time
	float ref = reference[0];				// Position reference
	float pos = sensor_->getValue();		// Position
	float error = ref - pos;				// Position error
	float speed;							// Speed
	float elapsed_time;						// Elapsed time in seconds

	// If last position is not yet defined
	if (last_pos_ == 0)
	{
		// Set last update time to current time
		clock_gettime(CLOCK_MONOTONIC, &current_time);
		last_time_ = current_time;

		// Set last position to current position
		last_pos_ = pos;
	}
	
	// Set elapsed time
	elapsed_time = current_time.tv_sec - last_time_.tv_sec;
	elapsed_time += (current_time.tv_nsec - last_time_.tv_nsec) / 1000000000;

	// Set speed
	speed = (pos - last_pos_) /  elapsed_time;

	// If error is above upper threshold
	if (error > THRESHOLD)
	{
		// Actuate
		//std::cout << "Above threshold: " << p_ * error + d_ * speed + TORQUE_CONSTANT * STARTING_TORQUE + g_torque_ << std::endl;
		actuator_->setValue(p_ * error + d_ * speed + TORQUE_CONSTANT * STARTING_TORQUE);
	}
	// If error is under lower threshold
	else if (error < - THRESHOLD)
	{
		// Actuate
		actuator_->setValue(p_ * error + d_ * speed + TORQUE_CONSTANT * STARTING_TORQUE);
	}
	// If error is within thresholds
	else
	{
		// Set 0
		actuator_->setValue(0);
	}

	
}