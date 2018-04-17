// Declarations
#include "robot.h"

// System
#include <cstring>				// memcpy
#include <iostream>				// std::cout, std::endl
#include <unistd.h>				// usleep

// Tools
#include "tools.h"				// messageToReference

// Maximum variable name length
#define MAX_VAR_NAME_LENGTH 16			// Should be the same as on the user PC

// Controller sample rate in microseconds
#define CONTROLLER_SAMPLE_RATE 1000		// 1000 us -> 10 ms

// Sensor sample rate in microseconds
#define SENSOR_SAMPLE_RATE 10000		// 10000 us -> 100 ms

namespace Robot
{
	// Initialize the robot
	void init(char* ip, int port)
	{
		// Initialize peer
		peer = new Peer(ip, port);

		// Initialize PRU
		pru = new PRU();
	}

	// Adds an encoder
	void addEncoder(std::string name, int output_location, float dist_per_count, bool invert)
	{
		// Initialize encoder
		Encoder* encoder;
		

		// If the encoder count should be inverted
		if (invert)
		{
			// Create encoder with switched polarity
			encoder = new Encoder(pru->getReference(output_location), - dist_per_count);
		}
		// If the encoder count should not be inverted
		else
		{
			// Create encoder
			encoder = new Encoder(pru->getReference(output_location), dist_per_count);
		}

		// Add encoder
		sensors.emplace(std::make_pair(name, encoder));
	}

	// Adds a motor
	void addMotor(std::string name, int input_location, int ccw_pin, int cw_pin, float max_speed, bool invert)
	{
		// Create motor
		Motor* motor = new Motor(pru->getReference(input_location), ccw_pin, cw_pin, max_speed, invert);

		// Add encoder
		actuators.emplace(std::make_pair(name, motor));
	}

	// Adds a proportional controller
	void addController(Controller* controller, std::string actuator_name, std::string sensor_name)
	{
		// Set actuator and sensor and of the controller
		controller->setActuator(actuators[actuator_name]);
		controller->setSensor(sensors[sensor_name]);

		// Add controller
		controllers.push_back(controller);
	}

	// Main function of the received thread
	void mainReceive()
	{
		// Loop indefinitely
		while(true)
		{
			// Receive reference
			receiveReference();
		}
	}

	// Main function of the send thread
	void mainSend()
	{
		// Sample the sensors and send to the user PC
		sampleSensors();
	}

	// Receive a reference from the user PC, to be started as a new thread
	void receiveReference()
	{
		char* msg;		// Holds message

		// Wait for a message
		msg = peer->receiveMessage();

		// If message if valid
		if (msg != INVALID_MESSAGE)
		{
			reference = messageToReference(msg);	
		}
	}

	// Runs the robot
	void run()
	{
		std::cout << "Waiting for an initial reference...";
		receiveReference();
		std::cout << "received!" << std::endl;

		// Create threads
		std::thread receive_thread(mainReceive);
		std::thread send_thread(mainSend);

		// Loop indefinitely
		std::cout << "Running robot..." << std::endl;
		while (true)
		{
			// For each controller
			for (Controller* controller : controllers)
			{	
				// Run controller
				controller->control(reference);
			}

			// wait for 0.1 seconds
			usleep(CONTROLLER_SAMPLE_RATE);
		}
	}

	// Samples the sensors and sends them over the UDP socket
	void sampleSensors()
	{
		int n_sensor;											// Amount of reference variables
		int n_sensor_size = sizeof(n_sensor);					// Length of variable that holds the amount of variables in bytes
		int sensor_size = MAX_VAR_NAME_LENGTH + sizeof(float);	// Length of reference variable in bytes

		int index;												// Holds the current index
		std::string name;										// Holds the current name
		float value;											// Holds the current value

		// Loop indefinitely
		while (true)
		{
			// Set current amoutn of sensors
			n_sensor = sensors.size();

			// Create socket message to send
			int messageLength = sizeof(n_sensor) + n_sensor * sensor_size;	// Required message length
			char msg[messageLength];										// Holds socket message

			// Copy amount of sensors to begin of message
			memcpy(&msg[0], &n_sensor, n_sensor_size);

			// Reset index
			index = n_sensor_size;

			// For each sensor
			for (const auto& pair : sensors)
			{
				// Extract name and value
				name = pair.first;
				value = pair.second->getValue();

				std::cout << "Value: " << value << std::endl;

				// Copy name to the message
				memcpy(&msg[index], &name[0], MAX_VAR_NAME_LENGTH * sizeof(char));

				// Update index
				index += MAX_VAR_NAME_LENGTH;

				// Copy value to message
				memcpy(&msg[index], &value, sizeof(float));

				// Update index
				index += sizeof(float);
			}
			
			// Send message
			peer->sendMessage(msg);

			// Wait for 0.1 seconds
			usleep(SENSOR_SAMPLE_RATE);
		}
	}
}