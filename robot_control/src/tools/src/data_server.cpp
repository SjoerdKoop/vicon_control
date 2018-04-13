// Components
#include "peer.h"		// Peer

// System
#include <cstring>		// memcpy
#include <iostream>		// std::cout, std::endl
#include <string>		// std::stoi
#include <unistd.h>		// usleep

// Tools
#include "tools.h"		// isValidIP, isValidPort

// Maximum variable name length
#define MAX_VAR_NAME_LENGTH 16

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
		// Create peer
		Peer* peer = new Peer(argv[1], std::stoi(argv[2]));

		// Set variables
		std::string name1 = "encoder0";
		std::string name2 = "test76";

		float value1 = -326.1f;
		float value2 = 8973.3323f;

		int n_var = 2;

		// Loop indefinitely
		while (true)
		{
			// Get user input
			std::cout << "Sending data..." << std::endl;

			// Initialize Message
			char msg[44];
			memset(msg, '\0', 44);

			// Populate message
			memcpy(&msg[0], &n_var, sizeof(int));
			memcpy(&msg[4], name1.c_str(), MAX_VAR_NAME_LENGTH);
			memcpy(&msg[20], &value1, sizeof(float));
			memcpy(&msg[24], name2.c_str(), MAX_VAR_NAME_LENGTH);
			memcpy(&msg[40], &value2, sizeof(float));

			// Send message
			peer->sendMessage(msg);

			// Sleep for 0.1 seconds
			usleep(100000);
		}
	}
	else 
	{
		// Show fatal error
		std::cout << "Please specify correct arguments: data_server <user IP address> <user PC port>" << std::endl;

		// Return failure status code
		return EXIT_FAILURE;
	}
}