// Components
#include "peer.h"			// Peer

// System
#include <cstdlib>			// EXIT_*, std::exit
#include <iostream>			// std::cout, std::endl
#include <signal.h>			// sigaction
#include <string>			// std::stoi

// Tools
#include "tools.h"			// isValidIP, isValidPort

// Checks whether provided arguments are correct
bool checkArguments(int argc, char* argv[])
{
	// If the number of arguments is correct
	if (argc == 3)
	{
		// If IP address and port are valid
		if (isValidIp(argv[1]) && isValidPort(argv[2]))
		{
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

// SIGINT handler
void handleUserInterrupt(int sig_num)
{
	// Exit the program
	std::exit(EXIT_SUCCESS);
}

// Main function
int main(int argc, char* argv[])
{
	// Check whether provided arguments are correct
	if (checkArguments(argc, argv))
	{
		// Create peer
		Peer* peer = new Peer(argv[1], std::stoi(argv[2]));

		// Register program for user interrupt
		struct sigaction act;
		act.sa_handler = handleUserInterrupt;
		sigaction(SIGINT, &act, NULL);
		
		char* msg;					// Holds received message
		std::vector<float> ref;		// Holds reference

		// Main loop
		while (true)
		{
			std::cout << "Ready to receive..." << std::endl;
			// Receive message
			msg = peer->receiveMessage();

			// Convert message to reference
			ref = messageToReference(msg);

			// Print reference
			std::cout << "Received reference: ";
			for (float var : ref)
			{
				std::cout << var << " ";
			}
			std::cout << std::endl;
		}
	}
	else 
	{
		// Show fatal error
		std::cout << "Please specify correct arguments: reference_client <user IP address> <user PC port>" << std::endl;

		// Return failure status code
		return EXIT_FAILURE;
	}
}