// PRU
// Always include stdint.h first 
#include <stdint.h>						// Type definitions for PRU headers
#include "am335x/pru_cfg.h"				// CT_CFG

// Resource table
// A resource table is required for the PRU to run
#include "resource_table_empty.h"		// Empty resource table

// Read encoders
#include "pru_encoder.h"				// PRUEncoder

// Main function
int main()
{
	// Enable OCP
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	// Create encoder with pins 0 and 1 and link memory location 0 to this encoder
	PRUEncoder* encoder0 = new PRUEncoder(0, 1, 0);

	// Loop infinitely
	while(1)
	{
		// Update encoder
		encoder0->update();
	}
}
