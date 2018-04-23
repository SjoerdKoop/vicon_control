// Declarations
#include "sensors.h"

// System
#include <string>		// std::string

// Path of the GPIO devices
#define GPIO_PATH "/sys/class/gpio"

// Constructor
PRUSensor::PRUSensor(int* output_location)
{
	// Set output memory location
	output_ = output_location;
}

// Constructor
Encoder::Encoder(int* output_location, float dist_per_count)
	: PRUSensor(output_location)
{
	// Set distance per count
	dist_per_count_ = dist_per_count;
}

// Gets the value of the output
float Encoder::getValue()
{
	return *output_ * dist_per_count_;
}


// Constructor
Hall::Hall(int pin)
{
	std::string output_string;	// Holds output pin string

	// Create strings
	output_string.append(GPIO_PATH).append("/gpio").append(std::to_string(pin)).append("/value");

	// Open value file at begin of the file
	output_file_.open(output_string.c_str());
}


// Gets the value of the output
float Hall::getValue()
{
	int value;			// Holds value

	// Reset file to begin
	output_file_.clear();
	output_file_.seekg(0, std::ios::beg);

	// Get value
	output_file_ >> value;
	
	return value;
}