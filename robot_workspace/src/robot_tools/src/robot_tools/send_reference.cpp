// Robot tools
#include "robot_tools/tools.h"

// System
#include <string>					// std::stoi

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
	// Checks whether provided arguments are correct
	if (checkArguments(argc, argv))
	{
		// Create reference array and fill it
		float ref[argc - 3];
		for (int i = 3; i < argc; i++)
		{
			ref[i - 3] = std::stoi(argv[3]);
		}
	}
}