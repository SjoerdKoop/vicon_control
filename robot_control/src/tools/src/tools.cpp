// Declarations
#include "tools.h"

// System
#include <arpa/inet.h>					// AF_INET, inet_pton, sockaddr_in
#include <cstdio>						// std::sscanf
#include <math.h>						// rintf
#include <iostream>						// std::cout, std::endl
#include <cstring>						// memcpy

// Checks whether argument is a valid IP address
bool isValidIp(char* ip)
{
	sockaddr_in address;
	int validAddress = inet_pton(AF_INET, ip, &(address.sin_addr));

	if (validAddress < 1)
	{
		return false;
	}
	else
	{
		return true;
	}
}

// Checks whether argument is a valid port
bool isValidPort(char* port)
{
	return isValidPositiveInteger(port);
}


// Checks whether argument is a positive integer
bool isValidPositiveInteger(char* arg)
{
	float tmp;
	int validNumber = std::sscanf(arg, "%f", &tmp);

	if (validNumber < 1)
	{
		return false;
	}
	else 
	{
		if (tmp != rintf(tmp))
		{
			return false;
		}
		else
		{
			if (tmp < 0)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
	}
}

// Converts socket message to reference
std::vector<float> messageToReference(char* msg)
{
	int n_var;							// Amount of reference variables
	int n_var_size = sizeof(n_var);		// Length of variable that holds the amount of variables in bytes
	int var_size = sizeof(float);		// Length of reference variable in bytes
	
	// Get the number of reference variables
	memcpy(&n_var, &msg[0], n_var_size);

	std::vector<float> ref;				// Constructed reference array
	
	// Reserve memory
	ref.reserve(n_var);

	float var;							// Current variable

	// For each reference variable
	for (int i = 0; i < n_var; i++)
	{
		// Get variable from message
		memcpy(&var, &msg[n_var_size + i * var_size], var_size);

		// Add variable to vector
		ref.push_back(var);
	}

	return ref;
}