// ROS
#include "ros/ros.h"						// ros::*, ROS_*

// Vicon tools
#include "vicon_tools/clients.h"			// ObjectClient
#include "vicon_tools/general_tools.h"		// isValidIp, isValidPort

// Checks whether provided arguments are correct
bool checkArguments(int argc, char* argv[])
{
	// If the number of arguments is not correct
	if (argc != 3)
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
	}

	// If all checks were succeful,return true
	return true;
}

int main(int argc, char* argv[])
{
	// Initialize ROS
	ros::init(argc, argv, "vicon_objects");

	// Checks whether provided arguments are correct
	if (checkArguments(argc, argv))
	{
		// Create client
		ObjectClient* object_client = new ObjectClient();

		if (object_client->connect(argv[1], argv[2]))
		{
			object_client->run();

			// Return standard status code
			return EXIT_SUCCESS;
		}
		else 
		{
			// Give fatal error
			ROS_FATAL("Could not connect to the Vicon datastream server");

			// Return failure status code
			return EXIT_FAILURE;
		}
	}
	else 
	{
		// Give fatal error
		ROS_FATAL("Please specify correct arguments: rosrun vicon objects <Vicon datastream IP address> <Vicon datastream port>");

		// Return failure status code
		return EXIT_FAILURE;
	}

}