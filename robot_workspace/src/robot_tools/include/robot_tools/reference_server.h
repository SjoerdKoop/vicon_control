#ifndef ROBOT_TOOLS_REFERENCE_SERVER_H
#define ROBOT_TOOLS_REFERENCE_SERVER_H

// Robot tools
#include "robot_tools/peer.h"				// Peer

// ROS
#include "ros/ros.h"						// ros::*
#include "std_msgs/Float32MultiArray.h"		// std_msgs::Float32MultiArray

// Client that listens to robot sensor data
class ReferenceServer
{
	public:
		// Constructor
		ReferenceServer();

		// Destructor
		~ReferenceServer();
    
		// Runs the controller
		void run(Peer* peer);

		static Peer* peer_;					// Static reference to the global peer

	private:	
		ros::Subscriber reference_sub_;		// Subscriber to reference_update

		// Converts ROS reference_update message to socket message
		static void onReferenceUpdate(const std_msgs::Float32MultiArray::ConstPtr& reference);
};

#endif // ROBOT_TOOLS_REFERENCE_SERVER_H