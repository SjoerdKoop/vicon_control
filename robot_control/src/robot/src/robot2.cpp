// Declarations
#include "robot.h"

#include "pru.h"
#include "sensor.h"

// System
#include <cstring>		// memcpy
#include <fcntl.h>		// open, O_RDWR, O_SYNC
#include <iostream>		// std::cout, std::endl
#include <math.h>		// M_PI
#include <signal.h>		// sigaction, SIGINT
#include <sys/mman.h>		// MAP_SHARED, mmap, PROT_READ, PROT_WRITE
#include <unistd.h>		// usleep

// Maximum variable name length
#define MAX_VAR_NAME_LENGTH 16			// Should be the same as on the user PC

// PRU memory addresses
#define PRU_ADDR 0x4A300000			// Address of the PRU-ICSS on the Sitara AM335x
#define PRU_SHARED_MEMORY 0x00010000		// Address of the shared memory block of the PRU-ICSS
#define PRU_LENGTH 0x00080000			// Length of the PRU-ICSS memory block

/*
// Constructor
Robot::Robot(char* ip, int port, Parameters1DOF parameters) {
	// Set parameters
	ccwPinNumber = parameters.ccwPinNumber;
	cwPinNumber = parameters.cwPinNumber;
	distancePerCount = parameters.distancePerCount;
	maxSpeed = parameters.maxSpeed;

	// Initialize peer
	peer = new Peer(ip, port);

	// Initialize shared memory
	initSharedMemory();

	// Initialize motor control pins
	initMotorPins();

	// Initialize member variables
	position = 0;
	reference = 0;

	// Robot should terminate
	shouldTerminate = false;

	PRU* pru = new PRU();

	Sensor* encoder = new Sensor("Encoder1", pru->getReference(0));

}
*/

// Destructor
Robot::~Robot() {
	// Notifiy termination
	shouldTerminate = true;

	// Stops the robot
	stop();

	// Close pin value files
	fclose(cwFile);
	fclose(ccwFile);

	// Delete peer
	delete peer;

	// Wait for threads to exit
	if (encoderThread.joinable()) {
		encoderThread.join();
	}

	if (referenceThread.joinable()) {
		referenceThread.join();
	}
}

// Stops the robot
void Robot::stop() {
        // Set speed to 0
        setSpeed(0);

        // Disable clockwise input
        fprintf(cwFile, "%d", 0);
        fflush(cwFile);

        // Disable counter clockwise input
        fprintf(ccwFile, "%d", 0);
        fflush(ccwFile);
}

// Initializes motor control pins
void Robot::initMotorPins() {
	// Open export file to export used pins
	FILE* exportFile = fopen("/sys/class/gpio/export", "w");

	// Seek begin of file
	fseek(exportFile, 0, SEEK_SET);

	// Print pin numbers to export
	// P8_41 => GPIO72
	// P8_43 => GPIO70
	fprintf(exportFile, "%d", cwPinNumber);
	fprintf(exportFile, "%d", ccwPinNumber);

	// Flush file and close
	fflush(exportFile);
	fclose(exportFile);

	// Open pin value files at begin of the files
	cwFile = fopen("/sys/class/gpio/gpio74/value", "w");
	ccwFile = fopen("/sys/class/gpio/gpio72/value", "w");
}

// Initializes shared memory
void Robot::initSharedMemory() {
        // Reset shared memory variable
        sharedMemory = NULL;

        // Open memory file for mapping
        int fd = open("/dev/mem", O_RDWR | O_SYNC);

        // If memory file cannot be opened
        if (fd == -1) {
                // Return error and exit
                std::cout << "ERROR: Could not open /dev/mem. Run as root or with \"sudo\"." << std::endl;
                std::exit(EXIT_FAILURE);
        }

        // Initialize PRU memory
        int* pru = (int*) mmap(0, PRU_LENGTH, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);

        // Map PRU memory to shared memory
        sharedMemory = pru + PRU_SHARED_MEMORY / 4;
}

// Moves the robot down at a given speed
void Robot::moveDown(float speed) {
	// Set speed
	setSpeed(speed);

	// Disable clockwise input
	fprintf(cwFile, "%d", 0);
	fflush(cwFile);

	// Enable counter clockwise input
	fprintf(ccwFile, "%d", 1);
	fflush(ccwFile);
}

// Moves the robot up at a given speed
void Robot::moveUp(float speed) {
        // Set speed
        setSpeed(speed);

        // Disable counter clockwise input
        fprintf(ccwFile, "%d", 0);
        fflush(ccwFile);

        // Enable clockwise input
        fprintf(cwFile, "%d", 1);
        fflush(cwFile);

}

// Runs the robot
void Robot::run() {
	// Start threads
	encoderThread = std::thread(readEncoder, this);
	referenceThread = std::thread(receiveReference, this);

	reference = 0.05;

	while(!shouldTerminate) {
		// Sleep a little to not take all system resources
		usleep(1000);
	}
}

// Static function reading and sending encoder values, to be started as a new thread
void Robot::readEncoder(Robot* robot) {
	char msg[MAX_VAR_NAME_LENGTH + sizeof(float)];					// Holds message
	char name[MAX_VAR_NAME_LENGTH] = {'E', 'n', 'c', 'o', 'd', 'e', 'r', '1'};	// Holds name of the encoder

	// Loop indefinitely
	while(!robot->shouldTerminate) {
		// Get measured position from the encoder
		std::cout << "Reading encoder: " << encoder->read() << std::endl;
		robot->position = robot->sharedMemory[0] * robot->distancePerCount;

		// Create message to send
		memcpy(&msg[0], name, MAX_VAR_NAME_LENGTH * sizeof(char));
		memcpy(&msg[MAX_VAR_NAME_LENGTH], &robot->position, sizeof(float));

		// Send message
		robot->peer->sendMessage(msg);

		// Wait 0.05 seconds
		usleep(50000);
	}
}

// Static function waiting to receive a reference from the user PC, to be started as a new thread
void Robot::receiveReference(Robot* robot) {
	char* msg;		// Holds message

	// Loop indefinitely
	while(!robot->shouldTerminate) {
		// Wait for a message
		msg = robot->peer->receiveMessage();

		if (msg != INVALID_MESSAGE) {
			std::cout << "Message received!" << std::endl;
			memcpy(&robot->reference, &msg[sizeof(int)], sizeof(float));
		}
	}
}

void Robot::setSpeed(float speed) {
	float dutyCycle;

	if (speed < 0) {
		speed = 0;
	}
	else if (speed > maxSpeed) {
		speed = maxSpeed;
	}

	// Escon 50/5 controller requires a duty cycle between 10% and 90%
	// Therefore duty cycle (in %)  =  dc_min + (dc_max - dc_min) * speed / speed_max
	sharedMemory[1] = 10 + 80 * speed / maxSpeed;
}
