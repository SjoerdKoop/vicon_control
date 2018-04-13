// Declarations
#include "controller.h"

#include <iostream>

// Sets the actuator
void Controller::setActuator(Actuator* actuator)
{
	// Set actuator
	actuator_ = actuator;
}

// Sets the sensor
void Controller::setSensor(Sensor* sensor)
{
	// Set sensor
	sensor_ = sensor;
}