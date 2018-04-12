#ifndef PRU_ENCODER
#define PRU_ENCODER

// Shared memory starting address
#define SHARED_MEMORY 0x00010000

// The PRU register __R31 for GPI
volatile register unsigned int __R31;

// Enum defining an encoder's state
enum EncoderState
{
	// Divided in four quadrants
	q1,
	q2,
	q3,
	q4
};

class Encoder
{
	public:
		//Constructor
		Encoder(int pin_A, int pin_B, int memory_location);
		
		// Updates the encoder
		void update();

	private:
		int state_;				// State of the encoder
		int mask_A_;			// Bit mask of input pin A
		int mask_B_;			// Bit mask of input pin B
		int* output_memory_;	// Location of the memory allocated for output

		// Gets the encoder's new state
		EncoderState getState();

		// Updates the encoder's count
		void updateCount(EncoderState new_state);
};

#endif // PRU_ENCODER
