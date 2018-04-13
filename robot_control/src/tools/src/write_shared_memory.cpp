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
	if (argc == 3)
	{
		// If argument is a valid positive integer
		if (isValidPositiveInteger(argv[1]) && isValidPositiveInteger(argv[2]))
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
	// Check whether provided arguments are correct
	if (checkArguments(argc, argv))
	{
		// Create PRU objects
		PRU* pru = new PRU();

		// Write to shared memory
		pru->setVariable(std::stoi(argv[1]), std::stoi(argv[2]));
	}
	else
	{
		// Show fatal error
		std::cout << "Please specify correct arguments: read_shared_memory <index> <value>" << std::endl;

		// Return failure status code
		return EXIT_FAILURE;
	}
}
