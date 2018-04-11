# Include directories, add all directories of used header files
INCLUDE_DIRS = include

# Source files: add all your source files
SOURCE_FILES = src/pwm_motor.cpp src/pwm.cpp

# Set firmware name, will be found in FIRMWARE_PATH
FIRMWARE_NAME = pwm_motor

################################################################################
####### DO NOT EDIT THE SECTION BELOW UNLESS YOU KNOW WHAT YOU ARE DOING #######

# Firmware destination path
FIRMWARE_PATH = ../../firmware

# PRU compiler
PRU_COMPILER_BIN = $(PRU_CGT)/bin/clpru			# PRU compiler binary
PRU_COMPILER_INCLUDE = ../../compiler/include	# PRU compiler header path
PRU_COMPILER_LIB = ../../compiler/lib			# PRU compiler library path

# PRU headers
PRU_INCLUDE = ../../include

# PRU linker command file
PRU_LINKER_COMMAND_FILE = ../../AM335x_PRU.cmd

# Set compiler flags
COMPILER_FLAGS = -q -O4 -mf5 $(INCLUDE_DIRS:%=-I%) -I$(PRU_COMPILER_INCLUDE)\
	-I$(PRU_INCLUDE) -i$(PRU_COMPILER_LIB)

# Set linker flags
LINKER_FLAGS = -z -l $(PRU_LINKER_COMMAND_FILE)

# Default target
default:
	@ # Creates the directory if it does not exists already
	@ [ -d $(FIRMWARE_PATH) ] || mkdir -p $(FIRMWARE_PATH)

	@ # Invoke the PRU compiler
	@ $(PRU_CGT)/bin/clpru $(COMPILER_FLAGS) $(SOURCE_FILES) $(LINKER_FLAGS)\
		 -o $(FIRMWARE_PATH)/$(FIRMWARE_NAME)

	@ # Clean up
	@ make clean

# Clean target
clean:
	@ # Remove all object files
	@ rm *.obj
################################################################################