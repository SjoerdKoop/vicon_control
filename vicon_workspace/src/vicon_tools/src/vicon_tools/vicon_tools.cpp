// Declarations
#include "vicon_tools/vicon_tools.h"

// System
#include <arpa/inet.h>			// AF_INET, inet_pton, sockaddr_in
#include <cstdio>				// std::sscanf
#include <math.h>				// rintf

// Checks whether argument is a valid IP address
bool isValidIp(char* ip) {
	sockaddr_in address;
	int validAddress = inet_pton(AF_INET, ip, &(address.sin_addr));

	if (validAddress < 1) {
		return false;
	}
	else {
		return true;
	}
}

// Checks whether argument is a valid port
bool isValidPort(char* port) {
	return isValidPositiveInteger(port);
}

// Checks whether argument is a valid posittive integer
bool isValidPositiveInteger(char* arg) {
	float tmp;
	int validNumber = std::sscanf(arg, "%f", &tmp);

	if (validNumber < 1) {
		return false;
	}
	else {
		if (tmp != rintf(tmp)) {
			return false;
		}
		else {
			if (tmp < 0) {
				return false;
			}
			else {
				return true;
			}
		}
	}
}