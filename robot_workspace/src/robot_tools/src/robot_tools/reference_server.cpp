// Declarations
#include "robot_tools/reference_server.h"

// Static reference to the peer
Peer* ReferenceServer::peer_;

// Constructor
ReferenceServer::ReferenceServer()
{
	// Initialize subscriber
	ros::NodeHandle n;
	reference_sub_ = n.subscribe<std_msgs::Float32MultiArray>("reference_update", 1, onReferenceUpdate);
}

// Destructor
ReferenceServer::~ReferenceServer()
{
	ROS_INFO("Shutting down!");
	// Shutdown subscriber
    reference_sub_.shutdown();
}

// Converts ROS reference_update message to socket message
void ReferenceServer::onReferenceUpdate(const std_msgs::Float32MultiArray::ConstPtr& reference)
{
	int n_ref = reference->data.size();						// Amount of reference variables
	int n_ref_size = sizeof(n_ref);							// Length of variable that holds the amount of variables in bytes
	int ref_size = sizeof(float);							// Length of reference variable in bytes

	int messageLength = sizeof(n_ref) + n_ref * ref_size;	// Required message length
	char msg[messageLength];								// Holds socket message

	// Copy amount of variables to begin of message
	memcpy(&msg[0], &n_ref, n_ref_size);

	// For each reference variable
	for (int i = 0; i < n_ref; i++)
	{
		// Append variable to message
		memcpy(&msg[n_ref_size + i * ref_size], &reference->data[i], ref_size);
	}

	// Send message
	peer_->sendMessage(msg);
}

// Runs the data client
void ReferenceServer::run(Peer* peer)
{
	// Set reference to peer
	ReferenceServer::peer_ = peer;

	// Only spin
	ros::spin();
}
