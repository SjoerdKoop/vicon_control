// Declarations
#include "vicon_peer/components/vicon.h"

// Buffer size
#define BUFFER_SIZE 512

// Quality threshold
#define QUALITY_THRESHOLD 0.0001


#include <unistd.h>

namespace Vicon {

// Constructor
ViconClient::ViconClient() {
	
}

// Calls connectToVicon if host is active
void ViconClient::connectIfActiveHost(int pingExitCode, QProcess::ExitStatus exitStatus) {
	// If host has been found
	if (pingExitCode == 0) {
		connectToDatastream();
	}
	else {
		log("Could not connect: No active host.");
		disconnectFromDatastream();
	}
}

// Connects to the Vicon datastream
void ViconClient::connectToDatastream() {
	char hostName[21];		// Holds host name
	Result::Enum result;	// Holds result from connect

	// Create SDK compatible host name
	sprintf(hostName, "%s:%d", Vicon::hostIP.toUtf8().constData(), Vicon::hostPort);

	// Attempt connection
	client = Client();
	std::cout << "Before connect" << std::endl;
	result = client.Connect(hostName).Result;
	std::cout << "After connect" << std::endl;

	// Check result
	switch (result) {
		case Result::ClientAlreadyConnected:
			// Log result
			log("Could not connect: Client already connected.");

			// Disconnect
			disconnectFromDatastream();
			break;
		case Result::ClientConnectionFailed:
			// Log result
			log("Could not connect: Connection failed.");

			// Disconnect
			disconnectFromDatastream();
			break;
		case Result::InvalidHostName:
			// Log result
			log("Could not connect: Invalid host name.");

			// Disconnect
			disconnectFromDatastream();
			break;
		case Result::Success:
			// Log result
			log("Connected! Listening to the datastream...");

			// Set general settings
			client.SetStreamMode(StreamMode::ServerPush);		// Streaming mode				
			client.SetBufferSize(BUFFER_SIZE);					// Buffer size

			// Let GUI know that client is connected
			//emit connected();

			//start();
			break;
		default:
			// Log result
			log("Could not connect: Unknown error.");

			// Disconnect
			disconnectFromDatastream();
			break;
	}
}

// Connects to the Vicon datastream
void connectToDatastream() {
	char hostName[21];		// Holds host name
	Result::Enum result;	// Holds result from connect

	// Create SDK compatible host name
	sprintf(hostName, "%s:%d", Vicon::hostIP.toUtf8().constData(), Vicon::hostPort);

	// Attempt connection
	//client = new Client();
	Client c;

	std::cout << "Before connect" << std::endl;
	//result = client->Connect(hostName).Result;
	result = c.Connect(hostName).Result;
	std::cout << "After connect" << std::endl;

	// Check result
	switch (result) {
		case Result::Success:
			// Log result
			std::cout << "Connected! Listening to the datastream..." << std::endl;

			usleep(1000000);
			c.Disconnect();
			std::cout << "Disconnected!" << std::endl;

			// Set general settings
			//client->SetStreamMode(StreamMode::ServerPush);		// Streaming mode				
			//client->SetBufferSize(BUFFER_SIZE);					// Buffer size

			// Let GUI know that client is connected
			//emit connected();

			//start();
			break;
		default:
			
			break;
	}
}

// Disconnects from the Vicon datastream
void ViconClient::disconnectFromDatastream() {
	//if (client->IsConnected().Connected) {

	//for (int i = 0; i < 50; i++) {
		//std::cout << "Before frame " << i << std::endl;
		//client->GetFrame();
	//}
	//emit stop();
	std::cout << "Disconnecting..." << std::endl;
	//client->Disconnect();

	// Disconnect datastream client
	//client->Disconnect();
	//std::cout << "After dc" << std::endl;
	//}

	// Let GUI know that client is disconnected
	//emit disconnected();
}

// Initializes the namespace
void init() {
	// Set up publishers
	ros::NodeHandle n;
	objectPub = n.advertise<vicon_peer::ros_object_array>("object_update", 1);
	removePub = n.advertise<vicon_peer::remove_objects>("object_remove", 1);

	// Set up ping process
	ping.setProgram("ping");

	// Connect signals
	//QObject::connect(&ping, SIGNAL(finished(int, QProcess::ExitStatus)), &viconClient, SLOT(connectIfActiveHost(int, QProcess::ExitStatus)));
}

// Terminates the namespace
void terminate() {
	// Shutdown publishers
	objectPub.shutdown();
	removePub.shutdown();
}

// Checks whether host is active, signals "connectIfActiveHost" of client
void checkHost(QString ip, int port) {
	// Save host information
	hostIP = ip;
	hostPort = port;

	// Run ping command to check whether a computer with given IP address is running
	// Program will hang indefinitely when trying to connect to an inactive host
	ping.setArguments(QStringList() << "-w1 " << hostIP);
	ping.start();
	connectToDatastream();
}
}