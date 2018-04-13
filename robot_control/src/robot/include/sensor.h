#ifndef SENSORS_H
#define SENSORS_H

// Class defining a sensor
class Sensor
{
	public:
		// Constructor
		Sensor(int* output_location);
	
		// Gets the value of the output
		virtual float getValue() = 0;

	protected:
		int* output_;			// Sensor output
};

// Class defining an Encoder
class Encoder : public Sensor
{
	public:
		// Constructor
		Encoder(int* output_location, float dist_per_count);

		// Gets the value of the output
		float getValue() override;

	private:
		float dist_per_count_;		// Distance per count
};

#endif // SENSORS_H