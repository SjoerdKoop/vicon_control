// Declarations
#include "actuator.h"

// Constructor
Actuator::Actuator(int* input_location)
{
	// Set output memory location
	input_ = input_location;
}

// Constructor
Motor::Motor(int* input_location, int ccw_pin, int cw_pin, float max_speed)
	: Actuator(input_location)
{
	// Open export file to export used pins
	FILE* export_file = fopen("/sys/class/gpio/export", "w");

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
	ccw_file = fopen("/sys/class/gpio/gpio74/value", "w");
	cw_file = fopen("/sys/class/gpio/gpio72/value", "w");

	// Set maximum speed
	maximum_speed_ = max_speed;
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

// Sets the speed
void Motor::setValue(float value)
{
	float dutyCycle;		// Holds the duty cycle

	value = - value;

	// If speed is positive
	if (value > 0)
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
	*input_ = 10 + 80 * value / maximum_speed_;
}