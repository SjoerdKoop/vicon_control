// PRU
// Always include stdint.h first 
#include <stdint.h>						// Type definitions for PRU headers
#include "am335x/pru_cfg.h"				// CT_CFG
#include "am335x/pru_iep.h"				// CT_IEP

// Resource table
// A resource table is required for the PRU to run
#include "resource_table_empty.h"		// Empty resource table

// PWM motor
#include "pru_pwm.h"					// PRUPWM

// Resets the IEP counter
void resetIEP()
{
	// Set counter to 0
	CT_IEP.TMR_CNT = 0x0;

	// Enable counter
	CT_IEP.TMR_GLB_CFG = 0x11;
}

// Reads from the IEP counter
int readIEP()
{
	// Return current count
	return CT_IEP.TMR_CNT;
}

// Main function
int main()
{
	// Enable OCP
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	// Cycle counter
	int cycles;

	// Create PWM with pin 0 and link memory location 1 to this PWM
	PRUPWM* pwm0 = new PRUPWM(0, 1);

	// Loop infinitely
	while(1)
	{
		// Reset IEP clock
		resetIEP();

		cycles = 0;

		// Wait for IEP clock to complete a cycle
		while(cycles < DEFAULT_PERIOD)
		{
			// Update PWM
			pwm0->update(cycles);

			// Update elapsed cycles
			cycles = readIEP();
		}
	}
}
