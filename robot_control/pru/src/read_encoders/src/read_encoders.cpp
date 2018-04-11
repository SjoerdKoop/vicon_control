// PRU
// Always include stdint.h first 
#include <stdint.h>						// Type definitions for PRU headers
#include "am335x/pru_cfg.h"				// CT_CFG
#include "am335x/pru_iep.h"				// CT_IEP

// Resource table
// A resource table is required for the PRU to run
#include "resource_table_empty.h"		// Empty resource table

// Read encoders
#include "encoder.h"

// Resets the IEP counter
void resetIEP() {
	// Set counter to 0
	CT_IEP.TMR_CNT = 0x0;

	// Enable counter
	CT_IEP.TMR_GLB_CFG = 0x11;
}

// Reads from the IRP counter
int readIEP() {
	// Return current count
	return CT_IEP.TMR_CNT;
}

// Main function
int main(void) {
	// Enable OCP
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	// Shared memory
	volatile int* shared_memory = (volatile int*) SHARED_MEMORY;

	// Create encoder with pins 0 and 1 and link memory location 0 to this encoder
	Encoder* encoder0 = new Encoder(0, 1, 0);

	// Loop infinitely
	while(1) {
		// Reset IEP
		resetIEP();

		// Update encoder
		encoder0->update();
		
		shared_memory[2] = readIEP();
	}
}
