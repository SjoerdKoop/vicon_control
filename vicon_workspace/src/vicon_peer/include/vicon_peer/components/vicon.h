#ifndef VICON_PEER_VICON_H
#define VICON_PEER_VICON_H

// Qt
#include <QObject>							// QObject
#include <QProcess>							// QProcess
#include <QThread>							// QThread

// ROS
#include "ros/ros.h"						// ros::*

// Vicon datastream SDK
#include "DataStreamClient.h"				// ViconDataStreamSDK::CPP::*

// Vicon peer
#include "vicon_peer/remove_objects.h"		// vicon_peer::remove_objects
#include "vicon_peer/ros_object_array.h"	// vicon_peer::ros_object_array

// Using Vicon Datastream SDK C++ namespace (makes code more readable)
using namespace ViconDataStreamSDK::CPP;


namespace Vicon {
	class ViconClient : public QObject {
		Q_OBJECT

		public:
			// Constructor
			ViconClient();

		private:
			Client client;		// Client

			// Connects to the Vicon datastream
			void connectToDatastream();

			// Disconnects from the Vicon datastream
			void disconnectFromDatastream();

		signals:
			void log(const QString& msg);

		public slots:
			// Calls connectToVicon if host is active
			void connectIfActiveHost(int pingExitCode, QProcess::ExitStatus exitStatus);

	};

	static QString hostIP;						// IP address of the host
	static int hostPort;						// Port of the host
	static ros::Publisher objectPub;			// Publisher of object_update
	static ros::Publisher removePub;			// Publisher of object_remove
	static QProcess ping;						// Ping process to check whether host is active
	static ViconClient viconClient;				// Vicon client

	static Client* client;

// Connects to the Vicon datastream
			void connectToDatastream();

	// Initializes the namespace
	void init();

	// Terminates the namespace
	void terminate();

	// Checks whether host is active, signals "connectIfActiveHost" of client
	void checkHost(QString ip, int port);
}

#endif // VICON_PEER_VICON_H