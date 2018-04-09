// Declarations
#include "robot_tools/tools.h"

// System
#include <arpa/inet.h>					// AF_INET, inet_pton, sockaddr_in
#include <cstdio>						// std::sscanf
#include <math.h>						// rintf

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
	float tmp;
	int validNumber = std::sscanf(port, "%f", &tmp);

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