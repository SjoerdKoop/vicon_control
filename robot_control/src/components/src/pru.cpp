// Declarations
#include "pru.h"

// System
#include <fcntl.h>			// open
#include <iostream>			// std::cout, std::endl
#include <sys/mman.h>		// mmap, MAP_*, O_*, PROT_*

// PRU memory characteristic
#define PRU_ADDRESS 0x4A300000
#define PRU_LENGTH 0x00080000
#define PRU_SHARED_MEMORY 0x00010000

// Constructor
PRU::PRU() {
        // Open memory file for mapping
        int fd = open("/dev/mem", O_RDWR | O_SYNC);

        // If memory file cannot be opened
        if (fd == -1) {
                // Return error and exit
                std::cout << "ERROR: Could not open /dev/mem." << std::endl;
                exit(EXIT_SUCCESS);
        }

        // Initializer PRU memory
        int* pru = (int*) mmap(0, PRU_LENGTH, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDRESS);

        // Map PRU memory to shared memory
        sharedMemory = pru + PRU_SHARED_MEMORY / 4; 
}

// Gets variable at index
int PRU::getVariable(int index) {
	return sharedMemory[index];
}

// Sets variable at index
void PRU::setVariable(int index, int value) {
	sharedMemory[index] = value;
}
