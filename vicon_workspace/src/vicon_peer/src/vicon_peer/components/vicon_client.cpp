// Declarations
#include "vicon_peer/components/vicon_client.h"

// Vicon peer
#include "vicon_peer/ros_object.h"
#include "vicon_peer/ros_object_array.h"

#include <iostream>
#include <unistd.h>
#include <string>			// std::string

// Buffer size
#define BUFFER_SIZE 512

// Quality threshold
#define QUALITY_THRESHOLD 0.0001

// Constructor
ViconClient::ViconClient() {
	// Set up publishers
	ros::NodeHandle n;
	objectPub = n.advertise<vicon_peer::ros_object_array>("object_update", 1);
	removePub = n.advertise<vicon_peer::remove_objects>("object_remove", 1);

	// Set up ping process
	ping.setProgram("ping");
	connect(&ping, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(connectIfActiveHost(int, QProcess::ExitStatus)));

	// Create client
	client.reset(new Client());
shouldRun = false;
	// Start client disconnected
	disconnectFromDatastream();

	// Connect signals to slots
	connect(this, SIGNAL(stop()), this, SLOT(terminate()));
}

// Destructor
ViconClient::~ViconClient() {
	// Delete client
	//client->Disconnect();
	//client.reset();

	// Notify that the client should stop
	emit stop();
	
	// Wait for running thread to terminate
	//runThread->wait();

	// Shutdown publisher
	objectPub.shutdown();

}

// Checks whether host is active
void ViconClient::checkHost(QString ip, int port) {
	log("Connecting to " + ip + " at port " + QString::number(port) + "...");

	// Save host information
	hostIP = ip;
	hostPort = port;

	// Run ping command to check whether a computer with given IP address is running
	// Program will hang indefinitely when trying to connect to an inactive host
	ping.setArguments(QStringList() << "-w1 " << hostIP);
	ping.start();
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
		result = Result::Enum::ClientConnectionFailed;
		// Create SDK compatible host name
		sprintf(hostName, "%s:%d", hostIP.toUtf8().constData(), hostPort);

		// Attempt connection
		result = client->Connect(hostName).Result;

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
				client->SetStreamMode(StreamMode::ServerPush);								// Streaming mode				
				client->SetBufferSize(BUFFER_SIZE);											// Buffer size

				// Let GUI know that client is connected
				emit connected();

				start();
				break;
			default:
				// Log result
				log("Could not connect: Unknown error.");

				// Disconnect
				disconnectFromDatastream();
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
	emit stop();
	std::cout << "Disconnecting..." << std::endl;
	//client->Disconnect();

	// Disconnect datastream client
	//client->Disconnect();
	//std::cout << "After dc" << std::endl;
	//}

	// Let GUI know that client is disconnected
	emit disconnected();
}

// Runs the client
void ViconClient::run() {
	Output_GetFrame ogf;						// Holds output of GetFrame
	vicon_peer::ros_object_array objectArray;	// Holds objects to send
	vicon_peer::remove_objects removeArray;		// Holds objects to remove

	while (shouldRun) {
		// Clear messages
		objectArray.objects.clear();
		removeArray.ids.clear();
		std::cout << "Running" << std::endl;
		// Wait for new frame
		ogf = client->GetFrame();
		
		// Check result
		switch (ogf.Result) {
			case Result::Success:
				if (client->IsSegmentDataEnabled().Enabled) {
					// Get message from objects
					objectsToMessage(&objectArray, &removeArray);
				}
				else {
					for (int i = 0; i < client->GetSubjectCount().SubjectCount; i++) {
						removeArray.ids.push_back(i);
					}
				}

				// Publish objects
				objectPub.publish(objectArray);
				removePub.publish(removeArray);

				// Spin once
				ros::spinOnce();
				break;
			case Result::NoFrame:
				// Log loss of connection
				log("Error: Could not acquire frame...");
				break;
		}
	}
}

// Starts the client
void ViconClient::start() {
	shouldRun = true;

	runThread = new QThread();
	this->moveToThread(runThread);
	connect(runThread, SIGNAL(started()), this, SLOT(run()));
	runThread->start();
}

// Switches modes
void ViconClient::switchMode(bool objects, int markers) {
	if (objects) {
		client->EnableSegmentData();
	}
	else {
		client->DisableSegmentData();
	}

	if (markers > 0) {
		nMarkers = markers;
		client->EnableUnlabeledMarkerData();
	}
	else {
		client->DisableUnlabeledMarkerData();
	}

	// Logging
	std::string msg;
	if (objects) {
		if (markers > 0) {
			msg = "Tracking objects and " + std::to_string(markers) + " singular markers.";
		}
		else {
			msg = "Tracking objects.";
		}
	}
	else {
		if (markers == 1) {
			msg = "Tracking " + std::to_string(markers) + " singular marker.";
		}
		else if (markers > 1) {
			msg = "Tracking " + std::to_string(markers) + " singular markers.";
		}
		else {
			msg = "Tracking off.";
		}
	}
		
	log(msg.c_str());
}

// Initiates termination
void ViconClient::terminate() {
	std::cout << "Terminate called" << std::endl;

	if (shouldRun) {
		if (runThread) {
			if (runThread->isRunning()) {
				std::cout << "Thread is running!" << std::endl;
				runThread->wait();
				std::cout << "Thread stopped!" << std::endl;
			}
			else {
				std::cout << "Thread not running!" << std::endl;
			}
		}
		else {
			std::cout << "No thread!" << std::endl;
		}
	}

	shouldRun = false;

}