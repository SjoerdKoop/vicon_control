#ifndef PROTO2_CONTROLLER_H
#define PROTO2_CONTROLLER_H

// Robot
#include "controller.h"

// Class defining the controller for prototype 1
class Proto2Controller : public Controller
{
	public:
		// Constructor
		Proto2Controller(float p);
		
		// Control action
		void control(std::vector<float> reference) override;

	private:
		float p_;						// Proportional gain
};

#endif // PROTO2_CONTROLLER_H