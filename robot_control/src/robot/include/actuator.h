#ifndef ACTUATOR_H
#define ACTUATOR_H

// System
#include <cstdio>		// FILE

// Class defining an actuator
class Actuator
{
	public:
		// Constructor
		Actuator(int* input_location);
	
		// Sets the value of the input
		virtual void setValue(float value) = 0;

	protected:
		int* input_;			// Actuator input
};

// Class defining a motor
class Motor : public Actuator
{
	public:
		// Constructor
		Motor(int* input_location, int ccw_pin, int cw_pin, float max_speed);

		// Destructor
		~Motor();

		// Sets the value to the input
		void setValue(float value) override;

	private:
		FILE* ccw_file;				// Counter clockwise pin file
		FILE* cw_file;				// Clockwise pin file
		float maximum_speed_;		// Maximum speed

		// Set the motor to turn clockwise	
		void setClockwise();

		// Set the motor to turn counter clockwise
		void setCounterClockwise();
};

#endif // ACTUATOR_H