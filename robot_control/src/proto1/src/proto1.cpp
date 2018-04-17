// Robot control
#include "robot.h"
#include "sensor.h"
#include "tools.h"
#include "pru.h"

#include "proto1_controller.h"

// System
#include <iostream>					// std::cout, std::endl

// Physical parameters
#define DISTANCE_PER_COUNT 0.000003		// 6 mm lead / (500 CPR * 4 quadrants)
#define MAX_SPEED 1.2					// 12000 RPM * 6 mm = 1.2 m/s

// Checks whether provided arguments are correct
bool checkArguments(int argc, char* argv[]) {
	// If the number of arguments is correct
	if (argc == 3) {
		// If IP address and port are valid
		if (isValidIp(argv[1]) && isValidPort(argv[2])) {
			return true;
		}
		else 
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

// Main function
int main(int argc, char* argv[])
{
	// Check whether provided arguments are correct
	if (checkArguments(argc, argv))
	{
		// Initialize robot
		Robot::init(argv[1], std::stoi(argv[2]));

		// Add actuators
		Robot::addMotor("motor0", 1, 49, 15, MAX_SPEED, true);

		// Add encoders
		Robot::addEncoder("encoder0", 0, DISTANCE_PER_COUNT, true);

		// Create controllers
		Proto1Controller* controller = new Proto1Controller(5.0f, 0.0f);

		// Add Controllers
		Robot::addController(controller, "motor0", "encoder0");

		// Runs the robot
		Robot::run();
	}
	else 
	{
		// Show fatal error
		std::cout << "Please specify correct arguments: proto1 <user IP address> <user PC port>" << std::endl;

		// Return failure status code
		return EXIT_FAILURE;
	}
}