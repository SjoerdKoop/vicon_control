// Declarations
#include "peer.h"

// System
#include <cstring>				// memset, strcmp
#include <iostream>				// std::cout, std::endl

// Constructor
Peer::Peer(char* ip, int port)
{
	// Create socket
	sock_ = socket(AF_INET , SOCK_DGRAM, IPPROTO_UDP);
		
	// Check whether socket is valid
	if (sock_ == -1)
	{
		std::cout << "Could not create socket!" << std::endl;
	}

	// Set robot IP and general address length
	robot_ip_ = ip;
	addr_length_ = sizeof(struct sockaddr_in);

	// Set sender address information
	send_addr_.sin_addr.s_addr = inet_addr(robot_ip_);
	send_addr_.sin_family = AF_INET;
	send_addr_.sin_port = htons(port);

	// Define communication over port <port>
	sockaddr_in selfAddr;
	selfAddr.sin_addr.s_addr = INADDR_ANY;
	selfAddr.sin_family = AF_INET;
	selfAddr.sin_port = htons(port);

	// Bind socket if possible, else throw error
	if (bind(sock_, (const sockaddr*)&selfAddr, addr_length_) == -1)
	{
		std::cout << "Could not bind socket to port " << port << "!" << std::endl;
	}
}

// Receives messages
char* Peer::receiveMessage()
{
	// Flush output and clear buffer
	fflush(stdout);
	memset(buffer_, '\0', BUFFER_SIZE);

	// Receive message and store sender address
	int nBytes = recvfrom(sock_, buffer_, BUFFER_SIZE, 0, (sockaddr*) &recv_addr_, &addr_length_);

	// Get IP addres of sender
	char ip_address[addr_length_];
	inet_ntop(AF_INET, &recv_addr_.sin_addr.s_addr, ip_address, addr_length_);

	// If the message came from the desired IP address
	if (strcmp(robot_ip_, ip_address) == 0)
	{
		// Return message
		return buffer_;
	}
	// If message came from an unknown sender
	else
	{
		// Return INVALID_MESSAGE
		return INVALID_MESSAGE;
	}
}

// Sends messages
void Peer::sendMessage(char* msg)
{
	// If sendto is unsuccesful
	if (sendto(sock_, msg, BUFFER_SIZE, 0, (sockaddr*) &send_addr_, addr_length_) == -1 )
	{
		// And the error is not ECONNREFUSED (no one is listening)
		if (errno != ECONNREFUSED)
		{
			std::cout << "Could not send message!" << std::endl;
		}
	}
}