#ifndef ROBOT_H
#define ROBOT_H

// Custom
//#include "parameters_1dof.h"	// Parameters1DOF
#include "peer.h"		// Peer
#include "sensor.h"
// System
#include <thread>		// std::thread

// Class acting as a robot
class Robot {
	public:
		// Constructor
		//Robot(char* ip, int port, Parameters1DOF parameters);

		// Destructor
		~Robot();

		// Runs the robot
		void run();

		// Stops the robot
		void stop();

	private:
		// Initializes motor control pins
		void initMotorPins();

		// Initialize shared memory
		void initSharedMemory();

		// Moves the robot down at a given speed
		void moveDown(float speed);

		// Moves the robot up at a given speed
		void moveUp(float speed);

		// Static function reading and sending encoder values, to be started as a new thread
		static void readEncoder(Robot* robot);

		// Static function waiting to receive a reference from the user PC, to be started as a new thread
		static void receiveReference(Robot* robot);

		// Sets the speed
		void setSpeed(float speed);

		// Eencoder reading thread
		std::thread encoderThread;

		// Reference receiving thread
		std::thread referenceThread;

		// Peer providing communication with the user PC
		Peer* peer;

		// Position of the robot
		float position;

		// Reference of the robot
		float reference;

		// Shared memory, needed to communicate with the PRU programs
		int* sharedMemory;

		// Pin value files for motor direction
		FILE* cwFile;
		FILE* ccwFile;

		// Holds boolean whether the robot should terminate (to communicate with threads)
		bool shouldTerminate;

		// Parameters
		int ccwPinNumber;
		int cwPinNumber;
		float distancePerCount;
		float maxSpeed;
};


Sensor* Robot::encoder;

#endif
