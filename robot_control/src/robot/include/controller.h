#ifndef CONTROLLER_H
#define CONTROLLER_H

// Robot control
#include "actuators.h"	// Actuator
#include "sensors.h"	// Sensor

// System
#include <vector>		// std::vector

// Class defining a controller
class Controller
{
	public:
		// Control action
		virtual void control(std::vector<float> reference) = 0;

		// Sets the actuator
		void setActuator(Actuator* actuator);

		// Sets the sensor
		void setSensor(Sensor* sensor);

	protected:
		Actuator* actuator_;	// Actuator
		Sensor* sensor_;		// Sensor
};

#endif // CONTROLLER_H