#include <stdint.h>				// needed for pru_iep.h
#include "am335x/pru_cfg.h"			// needed to initialize OCP
#include "am335x/pru_iep.h"			// needed for pru IEP counter
#include "resource_table_empty.h"


// Mask of the pin (0x1 => first bit (0))
#define MASK_PWM 0x1

// Period (in cycles @ 200 MHz)
#define PERIOD 200000		// 200000  => 1 kHz

// Shared memory address
#define SHARED_MEMORY 0x00010000

// PRU register __R30 for GPO
volatile register unsigned int __R30;

void resetIEP() {
	// Set counter to 0
	CT_IEP.TMR_CNT = 0x0;

	// Enable counter
	CT_IEP.TMR_GLB_CFG = 0x11;
}

int readIEP() {
	// Return current count
	return CT_IEP.TMR_CNT;
}

int main() {
	// Shared memory
	volatile int* sharedMemory = (volatile int*) SHARED_MEMORY;

	// Enable OCP
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	// Set duty cycle to 0
	sharedMemory[1] = 0;

	// Number of cycles the pulse should be high
	int nHigh;

	// Loop infinitely
	while(1) {
		// Reset IEP clock
		resetIEP();

		// Set pulse to high
		__R30 = MASK_PWM;

		// Update number of cycles on
		nHigh = sharedMemory[1] / 100.0f * PERIOD;

		// Wait for IEP clock to complete high time
		while(readIEP() < nHigh) {}

		// Set pulse to low
		__R30 = 0;

		// Wait for IEP clock to complete a cycle
		while(readIEP() < PERIOD) {}
        }
}
