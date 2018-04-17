#ifndef PROTO1_CONTROLLER_H
#define PROTO1_CONTROLLER_H

// Robot
#include "controller.h"

// System
#include <time.h>			// struct timespec,

// Class defining the controller for prototype 1
class Proto1Controller : public Controller
{
	public:
		// Constructor
		Proto1Controller(float p, float d);
		
		// Control action
		void control(std::vector<float> reference) override;

	private:
		float d_;						// Derivative gain
		float g_torque_;				// Gravitational torque
		struct timespec last_time_;		// Time of last update
		float p_;						// Proportional gain
		float last_pos_;				// Last position
};

#endif // PROTO1_CONTROLLER_H