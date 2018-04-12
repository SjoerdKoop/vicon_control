#ifndef TOOLS_TOOLS_H
#define TOOLS_TOOLS_H

// Checks whether argument is a valid IP address
bool isValidIp(char* ip);

// Checks whether argument is a valid port
bool isValidPort(char* port);

// Checks whether argument is a positive integer
bool isValidPositiveInteger(char* arg);

// Converts socket message to reference
float* messageToReference(char* msg);

#endif // TOOLS_TOOLS_H