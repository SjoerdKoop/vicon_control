#ifndef ROBOT_H
#define ROBOT_H

// Components
#include "peer.h"		// Peer
#include "pru.h"		// PRU

// Robot
#include "actuator.h"	// Actuator
#include "controller.h"	// Controller
#include "sensor.h"		// Sensor

// System
#include <map>			// std::map
#include <thread>		// std::thread
#include <vector>		// std::vector

namespace Robot
{
	static std::map<std::string, Actuator*> actuators;		// Actuators of the robot
	static std::vector<Controller*> controllers;			// Controllers of the robot
	static Peer* peer;										// Peer
	static PRU* pru;										// Intepretation of the PRU's
	static std::vector<float> reference;					// Reference
	static std::map<std::string, Sensor*> sensors;			// Sensors of the robot

	// Initialize the robot
	void init(char* ip, int port);

	// Adds an encoder
	void addEncoder(std::string name, int output_location, float dist_per_count, bool invert);

	// Adds a motor
	void addMotor(std::string name, int input_location, int ccw_pin, int cw_pin, float max_speed);

	// Adds a proportional controller
	void addController(Controller* controller, std::string actuator_name, std::string sensor_name);

	// Main function of the receive thread
	void mainReceive();

	// Main function of the send thread
	void mainSend();

	// Waits a for a reference
	void receiveReference();

	// Runs the robot
	void run();

	// Samples the sensors and sends them over the UDP socket
	void sampleSensors();
}

#endif
