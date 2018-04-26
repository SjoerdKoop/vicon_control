#ifndef ROBOT_CONTROLPEER_H
#define ROBOT_CONTROLPEER_H

// System
#include <arpa/inet.h>		// INADDR_ANY, sockaddr_in, socklen_t

// Buffer size (should the same as in the Vicon Tracker)
#define BUFFER_SIZE 512

// Return value of receiveMessage function when message is invalid
#define INVALID_MESSAGE 0x00

// Class defining a peer
class Peer {
	public:
		// Constructor
		Peer(char* robotIP, int port);

		// Receives messages
		char* receiveMessage();

		// Sends messages
		void sendMessage(char* msg);

	private:
		char buffer_[BUFFER_SIZE];			// Message buffer
		int sock_;							// Socket
		struct sockaddr_in recv_addr_;		// Receiving address information
		struct sockaddr_in send_addr_;		// Address information
		socklen_t addr_length_;				// Length of the sender structure in bytes
		char* robot_ip_;					// Valid IP of the robot
};

#endif // ROBOT_CONTROLPEER_H