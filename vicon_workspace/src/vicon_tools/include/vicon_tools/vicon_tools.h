#ifndef VICON_TOOLS_VICON_TOOLS_H
#define VICON_TOOLS_VICON_TOOLS_H

// System
#include <vector>	// std::vector

// Checks whether argument is a valid IP address
bool isValidIp(char* ip);

// Checks whether argument is a valid port
bool isValidPort(char* port);

// Checks whether argument is a valid posittive integer
bool isValidPositiveInteger(char* arg);

#endif // VICON_TOOLS_VICON_TOOLS_H