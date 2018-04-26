#ifndef ROBOT_CONTROL_SENSORS_H
#define ROBOT_CONTROL_SENSORS_H

// System
#include <fstream>		// std::ifstream

// Class defining a sensor
class Sensor
{
	public:
		// Gets the value of the output
		virtual float getValue() = 0;
};

// Class defining a sensor connected to a PRU
class PRUSensor : public Sensor
{
	public:
		// Constructor
		PRUSensor(int* output_location);
	
		// Gets the value of the output
		virtual float getValue() = 0;

	protected:
		int* output_;			// Sensor output
};

// Class defining an Encoder
class Encoder : public PRUSensor
{
	public:
		// Constructor
		Encoder(int* output_location, float dist_per_count);

		// Gets the value of the output
		float getValue() override;

	private:
		float dist_per_count_;		// Distance per count
};

// Class defining a Hall sensor
class Hall : public Sensor
{
	public:
		// Constructor
		Hall(int pin);

		// Gets the value of the output
		float getValue();

	private:
		std::ifstream output_file_;		// File of the output
};

// Class defining an Infrared sensor
class IRSensor : public Sensor
{
	public:
		// Constructor
		IRSensor(int pin);

		// Gets the value of the output
		float getValue();

	private:
		std::ifstream output_file_;		// File of the output
};

#endif // ROBOT_CONTROL_SENSORS_H