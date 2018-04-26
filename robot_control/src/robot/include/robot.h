#ifndef ROBOT_H
#define ROBOT_H

// Components
#include "peer.h"		// Peer
#include "pru.h"		// PRU

// Robot
#include "actuators.h"			// Actuator
#include "robot_controller.h"	// RobotController
#include "sensors.h"			// Sensor

// System
#include <map>			// std::map
#include <vector>		// std::vector

namespace Robot
{
	static std::map<std::string, Actuator*> actuators;		// Actuators of the robot
	static std::vector<RobotController*> controllers;		// Controllers of the robot
	static Peer* peer;										// Peer
	static PRU* pru;										// Intepretation of the PRU's
	static std::vector<float> reference;					// Reference
	static std::map<std::string, Sensor*> sensors;			// Sensors of the robot

	// Initialize the robot
	void init(char* ip, int port);

	// Adds an encoder
	void addEncoder(std::string name, int output_location, float dist_per_count, bool invert);

	// Adds a Hall sensor
	void addHallSensor(std::string name, int pin);

	// Adds a motor
	void addMotor(std::string name, int input_location, int ccw_pin, int cw_pin, float max_speed, bool invert);

	// Adds a controller
	void addController(RobotController* controller, std::string actuator_name, std::string sensor_name);

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

	// Set limits of an actuator
	void setActuatorLimits(std::string name, std::string lower_name, std::string upper_name);

}

#endif
