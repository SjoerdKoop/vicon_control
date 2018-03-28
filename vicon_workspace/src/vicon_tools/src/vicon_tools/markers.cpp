// ROS
#include "ros/ros.h"						// ros::*, ROS_*

// Vicon tools
#include "vicon_tools/clients.h"			// MarkerClient
#include "vicon_tools/general_tools.h"		// isValidIp, isValidPort, isValidPositiveInteger

// Checks whether provided arguments are correct
bool checkArguments(int argc, char* argv[])
{
	// If the number of arguments is not correct
	if (argc != 4)
	{
		return false;
	}
	else {
        // Check Vicon datastream IP address
		if (!isValidIp(argv[1]))
		{
			return false;
		}

		// Check Vicon datastream port
		if (!isValidPort(argv[2]))
		{
			return false;
		}

		// Check number of tracked markers
		if (!isValidPositiveInteger(argv[2]))
		{
			return false;
		}
	}

	// If all checks were succeful,return true
	return true;
}

int main(int argc, char* argv[]) {
	// Initialize ROS
	ros::init(argc, argv, "vicon_objects");

	// Checks whether provided arguments are correct
	if (checkArguments(argc, argv))
	{
		// Create client
		MarkerClient* marker_client = new MarkerClient(std::atoi(argv[3]));

		if (marker_client->connect(argv[1], argv[2]))
		{
			marker_client->run();

			// Return standard status code
			return EXIT_SUCCESS;
		}
		else 
		{
			// Return failure status code
			return EXIT_FAILURE;
		}
	}
	else 
	{
		// Show fatal error
		ROS_FATAL("Please specify correct arguments: rosrun vicon objects <Vicon datastream IP address> <Vicon datastream port> <Number of tracked objects>");

		// Return failure status code
		return EXIT_FAILURE;
	}

}