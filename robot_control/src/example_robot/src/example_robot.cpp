// Example robot
#include "example_robot_controller.h"	// ExampleRobotController (your controller)

// Robot control
#include "robot.h"						// Robot::*	(always include)

// Robot control tools
#include "tools.h"						// isValidIp, isValidPort (include to check arguments)

// System
#include <iostream>						// std::cout, std::endl	(include for user feedback)

// Physical parameters
#define DISTANCE_PER_COUNT 0.000003		// [m]
#define MAX_SPEED 1.2					// [m/s]

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

		// Add sensors
		Robot::addEncoder("encoder0", 0, DISTANCE_PER_COUNT, true);

		// Create controllers
		ExampleRobotController* controller = new ExampleRobotController(40.0f);

		// Add Controllers
		Robot::addController(controller, "motor0", "encoder0");

		// Runs the robot
		Robot::run();
	}
	else 
	{
		// Show fatal error
		std::cout << "Please specify correct arguments: example_robot <user IP address> <user PC port>" << std::endl;

		// Return failure status code
		return EXIT_FAILURE;
	}
}