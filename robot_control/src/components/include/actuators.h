#ifndef ROBOT_CONTROL_ACTUATORS_H
#define ROBOT_CONTROL_ACTUATORS_H

// Robot control
#include "sensors.h"	// Sensor

// System
#include <cstdio>		// FILE

// Class defining an actuator
class Actuator
{
	public:
		// Constructor
		Actuator(int* input_location);
	
		// Set the actuator's limits using sensors
		void setLimits(Sensor* lower, Sensor* upper);

		// Sets the value of the input
		virtual void setValue(float value) = 0;

	protected:
		Sensor* lower_;			// Sensor for lower limit
		bool has_limits_;		// Whether actuator has limits
		int* input_;			// Actuator input
		Sensor* upper_;			// Sensors for upper limit
};

// Class defining a motor
class Motor : public Actuator
{
	public:
		// Constructor
		Motor(int* input_location, int ccw_pin, int cw_pin, float max_speed, bool invert);

		// Destructor
		~Motor();

		// Sets the value to the input
		void setValue(float value) override;

	private:
		FILE* ccw_file_;			// Counter clockwise pin file
		FILE* cw_file_;				// Clockwise pin file
		bool invert_;				// Whether motor value should be inverted
		float maximum_speed_;		// Maximum speed

		// Set the motor to turn clockwise	
		void setClockwise();

		// Set the motor to turn counter clockwise
		void setCounterClockwise();

		// Sets the motor speed
		void setSpeed(float speed);

		// Stops the motor
		void stop();
};

#endif // ROBOT_CONTROL_ACTUATORS_H