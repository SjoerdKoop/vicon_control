// Delcarations
#include "pwm.h"

// Constructor
PWM::PWM(int pin, int memory_location)
{
	// Set pin
	pin_ = pin;

	// Set input memory location
	input_memory_ = (int*) (SHARED_MEMORY + memory_location * sizeof(int));

	// Update the PWM
	update(0);
}

// Updates the output signal of the PWM
void PWM::update(int cycles)
{
	// Updates the number of cycles the PWM should be high
	cycles_high_ = *input_memory_ / 100.0f * DEFAULT_PERIOD;

	// If the PWM should still give a high signal
	if (cycles < cycles_high_)
	{
		// Set corresponding bit to 1
		__R30 |= 1 << pin_;
	}
	// If the PWM should give a low signal
	else
	{
		// Set correpsonding bit to 0
		__R30 = (0 << pin_) & (~__R30);
	}
}
