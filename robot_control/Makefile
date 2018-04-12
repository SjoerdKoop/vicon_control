# Make flags
FLAGS = --no-print-directory -C

default: .FORCE
	@ # Makes projects with: 

	@ # Make binaries
	@ echo "--- Making ARM software ---"

	@ # Make components
	@ echo "Making components"
	@ $(MAKE) $(FLAGS) src/components

	@ # Make tools
	@ echo "Making tools"
	@ $(MAKE) $(FLAGS) src/tools

	@ # Add additional binaries to make here
	@ # echo "Making <name>"
	@ # $(MAKE) $(FLAGS) src/<project_directory>

	@ # Make firmware
	@ echo "--- Making PRU firmware ---"

	@ # Make pwm_motor
	@ echo "Making pwm_motor"
	@ $(MAKE) $(FLAGS) src/pwm_motor

	@ # Make read_encoders
	@ echo "Making read_encoders"
	@ $(MAKE) $(FLAGS) src/read_encoders

	@ # Add additional PRU firmware to make here
	@ # echo "Making <name>"
	@ # $(MAKE) $(FLAGS) src/<project_directory>
.FORCE: