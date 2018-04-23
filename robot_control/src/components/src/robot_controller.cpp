// Declarations
#include "robot_controller.h"

// Sets the actuator
void RobotController::setActuator(Actuator* actuator)
{
	// Set actuator
	actuator_ = actuator;
}

// Sets the sensor
void RobotController::setSensor(Sensor* sensor)
{
	// Set sensor
	sensor_ = sensor;
}