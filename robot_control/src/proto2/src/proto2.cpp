// Robot control
#include "robot.h"
#include "tools.h"

#include "proto2_controller.h"

// System
#include <iostream>					// std::cout, std::endl
#include <unistd.h>

// Physical parameters
#define DISTANCE_PER_COUNT 0.00000686274	// (7 cm diameter * pi) / (5.1 gear ratio * 500 CPR * 4 quadrants)
#define MAX_SPEED 3						// 12000 RPM * 7 cm diameter * pi / 5.1 gear ratio = 1.2 m/s

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
		Robot::addMotor("motor0", 1, 49, 15, MAX_SPEED, false);

		// Add encoders
		Robot::addEncoder("encoder0", 0, DISTANCE_PER_COUNT, true);

		// Add hall sensors
		Robot::addHallSensor("hall0", 66);
		Robot::addHallSensor("hall1", 67);

		// Set motor limits
		Robot::setActuatorLimits("motor0", "hall0", "hall1");

		// Create controllers
		Proto2Controller* controller = new Proto2Controller(2.5f);

		// Add Controllers
		Robot::addController(controller, "motor0", "encoder0");

		// Runs the robot
		Robot::run();
	}
	else 
	{
		// Show fatal error
		std::cout << "Please specify correct arguments: proto2 <user IP address> <user PC port>" << std::endl;

		// Return failure status code
		return EXIT_FAILURE;
	}
}