#ifndef PROTO1_CONTROLLER_H
#define PROTO1_CONTROLLER_H

// Robot
#include "controller.h"

// Class defining the controller for prototype 1
class Proto1Controller : public Controller
{
	public:
		// Constructor
		Proto1Controller(float p);
		
		// Control action
		void control(std::vector<float> reference) override;

	private:
		float p_;				// Proportional gain
};

#endif // PROTO1_CONTROLLER_H