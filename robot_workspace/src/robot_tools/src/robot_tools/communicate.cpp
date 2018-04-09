// Robot tools
#include "robot_tools/data_client.h"		// DataClient
#include "robot_tools/reference_server.h"	// RefrerenceServer
#include "robot_tools/tools.h"				// isValidIP, isValidPort
#include "robot_tools/peer.h"				// Peer

// ROS
#include "ros/ros.h"						// ros::*, ROS_*

// System
#include <string>							// std::stoi
#include <thread>							// std::thread

// Checks whether provided arguments are correct
bool checkArguments(int argc, char* argv[]) {
	// If the number of arguments is correct
	if (argc == 3) {
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

// Data client main function
void* dataClientMain(Peer* peer)
{
	// Create data client
	DataClient* data_client = new DataClient();

	// Main loop
	ROS_INFO("Data client running...");
	data_client->run(peer);
}

// Reference server main function
void* referenceServerMain(Peer* peer)
{
	// Create reference server
	ReferenceServer* reference_server = new ReferenceServer();

	// Main loop
	ROS_INFO("Reference server running...");
	reference_server->run(peer);
}

// Main function
int main(int argc, char* argv[])
{
	// Initialize ROS
	ros::init(argc, argv, "robot_communicate");

	// Checks whether provided arguments are correct
	if (checkArguments(argc, argv))
	{
		// Create peer
		Peer* peer = new Peer(argv[1], std::stoi(argv[2]));

		// Create data client thread
		std::thread data_client_thread(dataClientMain, peer);

		// Create reference server thread
		std::thread reference_server_thread(referenceServerMain, peer);

		// Wait for threads to finish
		data_client_thread.join();
		reference_server_thread.join();
	}
	else 
	{
		// Show fatal error
		ROS_FATAL("Please specify correct arguments: rosrun robot_tools communicate <robot IP address> <robot port>");

		// Return failure status code
		return EXIT_FAILURE;
	}
}