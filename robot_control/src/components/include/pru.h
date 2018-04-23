#ifndef ROBOT_CONTROL_PRU_H
#define ROBOT_CONTROL_PRU_H

// Class defining the PRU subsystem
class PRU {
	public:
		// Constructor
		PRU();

		// Gets the reference for the variable at index
		int* getReference(int index);

		// Gets variable at index
		int getVariable(int index);

		// Sets varaible at index
		void setVariable(int index, int value);

		// The shared memory between the processor and PRU's
		int* sharedMemory;
};

#endif // ROBOT_CONTROL_PRU_H