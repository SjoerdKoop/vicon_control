// Declarations
#include "actuator.h"

// System
#include <string>			// std::string

// Path of the GPIO devices
#define GPIO_PATH "/sys/class/gpio"

// Constructor
Actuator::Actuator(int* input_location)
{
	// Set output memory location
	input_ = input_location;
}

// Constructor
Motor::Motor(int* input_location, int ccw_pin, int cw_pin, float max_speed, bool invert)
	: Actuator(input_location)
{
	std::string ccw_string;		// Holds counter clockwise pin string
	std::string cw_string;		// Holds clockwise pin string
	std::string export_string;	// Holds export string

	// Create strings
	ccw_string.append(GPIO_PATH).append("/gpio").append(std::to_string(ccw_pin)).append("/value");
	cw_string.append(GPIO_PATH).append("/gpio").append(std::to_string(cw_pin)).append("/value");
	export_string.append(GPIO_PATH).append("/export");

	// Open export file to export used pins
	FILE* export_file = fopen(export_string.c_str(), "w");

	// Seek begin of file
	fseek(export_file, 0, SEEK_SET);

	// Print pin numbers to export
	// P8_41 => GPIO74
	// P8_43 => GPIO72
	fprintf(export_file, "%d", ccw_pin);
	fprintf(export_file, "%d", cw_pin);

	// Flush file and close
	fflush(export_file);
	fclose(export_file);

	// Open pin value files at begin of the files
	cw_file = fopen(cw_string.c_str(), "w");
	ccw_file = fopen(ccw_string.c_str(), "w");

	// Set maximum speed
	maximum_speed_ = max_speed;

	// Set inverted
	invert_ = invert;
}

// Destructor
Motor::~Motor()
{
	// Close pin files
	fclose(ccw_file);
	fclose(cw_file);
}

// Set the motor to turn clockwise	
void Motor::setClockwise()
{
	// Disable counter clockwise input
	fprintf(ccw_file, "%d", 0);
	fflush(ccw_file);

	// Enable clockwise input
	fprintf(cw_file, "%d", 1);
	fflush(cw_file);
}

// Set the motor to turn counter clockwise
void Motor::setCounterClockwise()
{
	// Disable clockwise input
	fprintf(cw_file, "%d", 0);
	fflush(cw_file);

	// Enable counter clockwise input
	fprintf(ccw_file, "%d", 1);
	fflush(ccw_file);
}

// Stops the motor
void Motor::stop()
{
	// Disable clockwise input
	fprintf(cw_file, "%d", 0);
	fflush(cw_file);

	// Disable counter clockwise input
	fprintf(ccw_file, "%d", 0);
	fflush(ccw_file);
}

// Sets the speed
void Motor::setValue(float value)
{
	float dutyCycle;		// Holds the duty cycle

	// Invert value if needed
	if (invert_)
	{
		value = - value;
	}

	// If speed should be set to 0
	if (value == 0)
	{
		// Stop motor
		stop();
	}
	// If speed is positive
	else if (value > 0)
	{
		// Set motor to turn clockwise
		setClockwise();

	}
	// If speed is negative
	else
	{	
		// Set motor to turn counter clockwise	
		setCounterClockwise();
		
		// Change polarity of speed
		value = - value;
	}

	// Clamp speed if neccesary
	if (value > maximum_speed_)
	{
		value = maximum_speed_;
	}
	
	// Escon 50/5 controller requires a duty cycle between 10% and 90%
	// Therefore duty cycle (in %)  =  dc_min + (dc_max - dc_min) * speed / speed_max
	*input_ = (float)(10.0f + 80.0f * value / maximum_speed_);
}