#ifndef VICON_PEER_VICON_CLIENT_H
#define VICON_PEER_VICON_CLIENT_H

// Qt
#include <QProcess>							// QProcess
#include <QThread>							// QThread

// ROS
#include "ros/ros.h"						// ros::*

// System
#include <memory>							// std::shared_ptr

// Vicon datastream SDK
#include "DataStreamClient.h"				// ViconDataStreamSDK::CPP::*

// Vicon peer
#include "vicon_peer/remove_objects.h"		// vicon_peer::remove_objects
#include "vicon_peer/ros_object_array.h"	// vicon_peer::ros_object_array

// Using Vicon Datastream SDK C++ namespace (makes code more readable)
using namespace ViconDataStreamSDK::CPP;

class ViconClient : public QObject {
	Q_OBJECT

	public:
		// Constructor
		ViconClient();

		// Destructor
		~ViconClient();

		// Set specific options
		//virtual void setOptions();

		// Checks whether host is active, signals "connectIfActiveHost"
		void checkHost(QString ip, int port);

		// Disconnects from the Vicon datastream
		void disconnectFromDatastream();

		// Starts the client
		void start();

		// Switches modes
		void switchMode(bool objects, int markers);

	private:
		std::shared_ptr<Client> client;		// Vicon datastream client
		QString hostIP;						// IP address of current host
		int hostPort;						// Port of the host
		ros::Publisher objectPub;			// Publisher of object_update
		ros::Publisher removePub;			// Publisher of object_remove
		QProcess ping;						// Ping process, used for checked whether given host is active

		QThread* runThread;
		bool shouldRun;

		int nMarkers;

		// Connects to the Vicon datastream
		void connectToDatastream();

		// Converts objects to message
		void objectsToMessage(vicon_peer::ros_object_array* objectArray, vicon_peer::remove_objects* removeArray);

	signals:
		// Signal when connection has been made
		void connected();

		// Signal when connection has been shut down or was invalid
		void disconnected();

		// Signals logging of a message
		void log(const QString& msg);

		// Signals stop
		void stop();

	public slots:
		// Runs the client
		void run();

	private slots:
		// Calls connectToVicon if host is active
		void connectIfActiveHost(int pingExitCode, QProcess::ExitStatus exitStatus);

		// Initiates termination
		void terminate();
};

#endif