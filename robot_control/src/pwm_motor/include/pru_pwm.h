#ifndef PRU_PWM
#define PRU_PWM

// Default period (in cycles @ 200 MHz)
#define DEFAULT_PERIOD 200000		// 200000 cycles => 1 kHz
//#define DEFAULT_PERIOD 200000000	// 200000000 cycles => 1 Hz

// Shared memory starting address
#define SHARED_MEMORY 0x00010000

// PRU register __R30 for GPO
volatile register unsigned int __R30;

// Class defining a Pulse Width Modulator for the PRU
class PRUPWM
{
	public:
		// Constructor
		PRUPWM(int pin, int memory_location);

		// Updates the output signal of the PWM
		void update(int cycles);
int cycles_high_;	// How many cycles the PWM has to give a high signal
		
	private:
		int pin_;			// Output pin
		int* input_memory_;	// Location of the memory allocated for input
};

#endif // PRU_PWM
