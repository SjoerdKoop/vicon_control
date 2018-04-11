// Robot
#include "pru.h"		// PRU

// System
#include <iostream>		// std::cout, std::endl
#include <unistd.h>		// usleep

// Tools
#include "tools.h"		// isValidPositiveInteger

// Checks whether provided arguments are correct
bool checkArguments(int argc, char* argv[])
{
	// If the number of arguments is correct
	if (argc == 2)
	{
		// If argument is a valid positive integer
		if (isValidPositiveInteger(argv[1]))
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

// Main function
int main(int argc, char* argv[])
{
	if (checkArguments(argc, argv))
	{
		// Store index
		int index = std::stoi(argv[1]);

		// Create PRU objects
		PRU* pru = new PRU();

		// Loop indefinitely (until user interrupt)
		while (true)
		{
			// Print user feedback
			std::cout << "Value at " << index << ": " << pru->getVariable(index) << std::endl;

			// Wait 0.1 seconds
			usleep(100000);
		}
	}
	else
	{
		// Show fatal error
		std::cout << "Please specify correct arguments: read_shared_memory <index>" << std::endl;

		// Return failure status code
		return EXIT_FAILURE;
	}
}
