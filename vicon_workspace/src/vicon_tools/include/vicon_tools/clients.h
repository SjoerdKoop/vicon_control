#ifndef VICON_TOOLS_CLIENTS_H
#define VICON_TOOLS_CLIENTS_H

// ROS
#include "ros/ros.h"							// ros::*

// Vicon datastream SDK
#include "DataStreamClient.h"               	// ViconDataStreamSDK::CPP::*

// Vicon tools
#include "vicon_tools/ros_object_array.h"		// vicon_glboal::ros_object_array
#include "vicon_tools/tracked_object.h"			// TrackedObject
#include "vicon_tools/vector3D.h"				// Vector3D

// Vicon client buffer size
#define BUFFER_SIZE 512

// Norm threshold for sending markers
#define NORM_THRESHOLD 1000

// Quality threshold for sending objects
#define QUALITY_THRESHOLD 0.0001

// Using Vicon Datastream SDK C++ namespace (makes code more readable)
using namespace ViconDataStreamSDK::CPP;

// Class connecting to vicon
class ViconClient 
{
	public:
		// Constructor
		ViconClient(int number_of_markers);

		// Destructor
		~ViconClient();

		// Connects to the datastream
		bool connect(char* ip, char* port);

		// Extracts desired data from the client
		// To be overwritten by children
		virtual vicon_tools::ros_object_array extractData();

		// Runs the client
		virtual void run();

	protected:
		Client* client_;								// Datastream client
		vicon_tools::ros_object_array object_array;		// Array of objects to publish on topic "object_update"

		// Generates ros_object_array from markers
		vicon_tools::ros_object_array getMarkers();

		// Generates ros_object_array from objects
		vicon_tools::ros_object_array getObjects();

	private:
		ros::Publisher pub_;							// Publisher
		std::vector<TrackedObject> objects_;			// Tracked objects

};

// Class defining a client that publishes markers and objects
class DualClient : public ViconClient
{
	public:
		// Constructor
		DualClient(int number_of_markers);

		// Runs the client
		void run() override;

	private:
		// Extracts desired data from the client
		vicon_tools::ros_object_array extractData() override;
};

// Class defining a client that publishes markers
class MarkerClient : public ViconClient
{
	public:
		// Constructor
		MarkerClient(int number_of_markers);

		// Runs the client
		void run() override;

		// Extracts desired data from the client
		virtual vicon_tools::ros_object_array extractData() override;
};

// Class defining a client that publishes objects
class ObjectClient : public ViconClient
{
	public:
		// Constructor
		ObjectClient();

		// Runs the client
		void run() override;

	private:
		// Extracts desired data from the client
		virtual vicon_tools::ros_object_array extractData() override;
};

#endif // VICON_TOOLS_CLIENTS_H