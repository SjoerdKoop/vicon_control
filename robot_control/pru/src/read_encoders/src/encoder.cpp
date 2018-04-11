// Declarations
#include "encoder.h"

// Constructor
Encoder::Encoder(int pin_A, int pin_B, int memory_location)
{
	// Set masks
	mask_A_ = 0x0001 << pin_A;
	mask_B_ = 0x0001 << pin_B;

	// Set output memory location and initialize to 0
	output_memory_ = (int*) (SHARED_MEMORY + memory_location * sizeof(int));
	*output_memory_ = 0;

	// Set state
	state_ = getState();
}

// Updates the encoder
void Encoder::update()
{
	// Get new state
	EncoderState new_state = getState();

	// Update count
	updateCount(new_state);
}

// Gets the encoder's new state
EncoderState Encoder::getState()
{
	// Read pins
	int A = (__R31 & mask_A_);
	int B = (__R31 & mask_B_);

	// Link state of inputs to quadrant
	if (A == mask_A_)
	{
		if (B == mask_B_)
		{
			return q3;
		}
		else
		{
			return q2;
		}
	}
	else
	{
		if (B == mask_B_)
		{
			return q4;
		}
		else
		{
			return q1;
		}
	}
}

// Updates the encoder's count
void Encoder::updateCount(EncoderState new_state)
{
	int change = 0;			// Holds the change in counts

	// Compare current and new states and set change accordingly
	switch(state_)
	{
		case q1:
			switch(new_state)
			{
				case q4:
					change = -1;
					break;
				case q2:
					change = 1;
					break;
			}
			break;
		case q2:
			switch(new_state)
			{
				case q1:
					change = -1;
					break;
				case q3:
					change = 1;
					break;
			}
			break;
		case q3:
			switch(new_state)
			{
				case q2:
					change = -1;
					break;
				case q4:
					change = 1;
					break;
			}
			break;
		case q4:
			switch(new_state)
			{
				case q3:
					change = -1;
					break;
				case q1:
					change = 1;
					break;
			}
			break;
	}

	// Update counts with the change
	*output_memory_ += change;

	// Set previous state to current state
	state_ = new_state;
}
