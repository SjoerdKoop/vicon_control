#ifndef PRU_H
#define PRU_H

class PRU {
	public:
		// Constructor
		PRU();

		// Gets variable at index
		int getVariable(int index);

		// Sets varaible at index
		void setVariable(int index, int value);

	private:
		// The shared memory between the processor and PRU's
		int* sharedMemory;
};

#endif
