// Robot tools
#include "robot_tools/peer.h"		// Peer
#include "robot_tools/tools.h"		// isValidIp, isValidPort

// ROS
#include "ros/ros.h"				// ros::init

// System
#include <iostream>					// std::cout, std::endl
#include <cstring>					// memcpy, std::stoi

// Checks whether provided arguments are correct
bool checkArguments(int argc, char* argv[]) {
	// If the number of arguments is correct
	if (argc > 3) {
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
	ros::init(argc, argv, "robot_tools_send_reference");

	// Checks whether provided arguments are correct
	if (checkArguments(argc, argv))
	{
		// Create peer
		Peer* peer = new Peer(argv[1], std::stoi(argv[2]));

		int n_ref = argc - 3;									// Amount of reference variables
		int n_ref_size = sizeof(n_ref);							// Length of variable that holds the amount of variables in bytes
		int ref_size = sizeof(float);							// Length of reference variable in bytes

		int message_length = sizeof(n_ref) + n_ref * ref_size;	// Required message length
		char msg[message_length];								// Holds socket message

		memset(msg, '\0', message_length);

		// Copy amount of variables to begin of message
		memcpy(&msg[0], &n_ref, n_ref_size);
		
		// For each reference variable
		for (int i = 0; i < n_ref; i++)
		{
			// Append variable to message
			float ref_val = std::atof(argv[3 + i]);
			memcpy(&msg[n_ref_size + i * ref_size], &ref_val, ref_size);
		}

		// Send message
		peer->sendMessage(msg);
	}
	else 
	{
		// Show fatal error
		std::cout << "Please specify correct arguments: rosrun robot_tools send_reference <robot IP address> <robot port> <ref0> <ref1> ..." << std::endl;

		// Return failure status code
		return EXIT_FAILURE;
	}
}