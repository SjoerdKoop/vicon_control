#ifndef ROBOT_TOOLS_DATA_CLIENT_H
#define ROBOT_TOOLS_DATA_CLIENT_H

// Robot tools
#include "robot_tools/data_update_array.h"	// robot_tools::data_update_array
#include "robot_tools/peer.h"				// Peer

// ROS
#include "ros/ros.h"						// ros::*

// Client that listens to robot sensor data
class DataClient
{
	public:
		// Constructor
		DataClient();

		// Destructor
		~DataClient();
    
		// Runs the controller
		void run(Peer* peer);

	private:
		ros::Publisher data_pub_;			// Publisher to data_update

		// Converts socket message to ROS data_update message
		robot_tools::data_update_array messageToDataUpdate(char* msg);    
};

#endif // ROBOT_TOOLS_DATA_CLIENT_H