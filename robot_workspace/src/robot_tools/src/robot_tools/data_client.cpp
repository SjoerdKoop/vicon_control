// Declarations
#include "robot_tools/data_client.h"

// Maximum variable name length
#define MAX_VAR_NAME_LENGTH 16

// Data variable type
#define VAR_TYPE float

// Constructor
DataClient::DataClient()
{
	// Initialize publisher
	ros::NodeHandle n;
	data_pub_ = n.advertise<robot_tools::data_update_array>("data_update", 1);
}

// Destructor
DataClient::~DataClient()
{
	// Shutdown publisher
    data_pub_.shutdown();
}

// Converts socket message to ROS data_update message
robot_tools::data_update_array DataClient::messageToDataUpdate(char* msg)
{
	robot_tools::data_update_array data_update_array;			// Holds ROS message
	
	// Message variables
	int n_var;													// Amount of  variables
	int n_var_size = sizeof(n_var);								// Length of variable that holds the amount of variables
	int var_value_length = sizeof(VAR_TYPE);					// Length of the value of the variable
	int var_size = MAX_VAR_NAME_LENGTH + var_value_length;		// Length of variable (name + value) in bytes

	// Loop variables
	robot_tools::data_update data_update;						// Holds ROS data update	
	char name[MAX_VAR_NAME_LENGTH];								// Holds name of current variable
	int name_index;												// Holds the index for current name
	int value_index;											// Holds the index for current value


	// Get number of variables
	memcpy(&n_var, &msg[0], n_var_size);

	// For each variable
	for (int i = 0; i < n_var; i++)
	{
		// Calculate indices
		name_index = n_var_size + i * var_size;
		value_index = name_index + MAX_VAR_NAME_LENGTH;

		// Get name
		memcpy(name, &msg[name_index], MAX_VAR_NAME_LENGTH);
		data_update.name = name;

		// Get value
		memcpy(&data_update.value, &msg[value_index], var_value_length);

		// Add to array
		data_update_array.updates.push_back(data_update);
	}

	// Return ROS message
	return data_update_array;
}

// Runs the data client
void DataClient::run(Peer* peer)
{
	robot_tools::data_update_array data_update_array;	// Holds ROS message
	char* msg;											// Holds received message

	// While ROS is running
	while(ros::ok())
	{
		// Wait for message from robot
		msg = peer->receiveMessage();

		if (msg != INVALID_MESSAGE)
		{
			// Convert received message to ROS message
			data_update_array = messageToDataUpdate(msg);

			// Publish data_update message
			data_pub_.publish(data_update_array);
		}
		// Spin once
		ros::spinOnce();        
	}
}