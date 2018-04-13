// Declarations
#include "sensor.h"

#include <iostream>

// Constructor
Sensor::Sensor(int* output_location)
{
	// Set output memory location
	output_ = output_location;
}

// Constructor
Encoder::Encoder(int* output_location, float dist_per_count)
	: Sensor(output_location)
{
	// Set distance per count
	dist_per_count_ = dist_per_count;
}

// Gets the value of the output
float Encoder::getValue()
{
	return *output_ * dist_per_count_;
}