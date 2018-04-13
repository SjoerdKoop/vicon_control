################################################################################
########## DO NOT EDIT THE SECTION UNLESS YOU KNOW WHAT YOU ARE DOING ##########
# Firmware and library destination paths
FW_PATH = ../../firmware
LIB_PATH = ../../lib

# ARM binaries
CXX = /usr/bin/arm-linux-gnueabihf-g++	# Compiler binary
AR = /usr/bin/arm-linux-gnueabihf-ar	# Archiver binary

# PRU compiler binary
CXX = $(PRU_CGT)/bin/clpru	

# PRU compiler paths
CXX_INCLUDE = $(PRU_CGT)/include	# PRU compiler header path
CXX_LIB = $(PRU_CGT)/lib			# PRU compiler library path

# PRU ICSS headers
ICSS_INCLUDE = $(PRU_ICSS)/include

# PRU linker command file
PRU_LINKER_COMMAND_FILE = $(PRU_ICSS)/AM335x_PRU.cmd

# Set compiler flags
PRU_INCLUDE_FLAGS = -I$(ICSS_INCLUDE) -I$(CXX_INCLUDE) -i$(CXX_LIB)
LINK_FLAGS = -z -l $(PRU_LINKER_COMMAND_FILE)
FLAGS = -q -O4 -mf5 $(INCLUDE_DIRS:%=-I%) $(PRU_INCLUDE_FLAGS) $(LINK_FLAGS)
################################################################################

# Include directories, add all directories of used header files
INCLUDE_DIRS = include

# Source files: add source files of targets
# <NAME>_SRCS = src/<src1>.cpp src/<src2>.cpp ...
PWM_MOTOR_SRCS= src/pwm_motor.cpp src/pru_pwm.cpp

# Library files: add library files of targets
# For include libfile1, use -lfile1
# <NAME>_LIBS = -lfile1 -lfile2 ...

# Default target
default: create clean

# Create target
create:
	@ # Creates the directories if they do not exist
	@ [ -d $(FW_PATH) ] || mkdir -p $(FW_PATH)
	@ [ -d $(LIB_PATH) ] || mkdir -p $(LIB_PATH)

	@ # Invoke the PRU compiler to create firmware and libraries:

	@ # For firmware to FW_PATH:
	@ # $(CXX) $(<name>_SRCS) $(FLAGS) -o $(FW_PATH)/<name>

	@ # For libraries to LIB_PATH:
	@ # $(CXX) -c $(FLAGS) $(<name>_SRCS) -o <name>.o
	@ # $(AR) crf $(LIB_PATH)/lib<name>.a -o <name>.o


	@ # Create pwm_encoder firmware
	@ $(CXX) $(PWM_MOTOR_SRCS) $(FLAGS) -o $(FW_PATH)/pwm_motor

# Clean target
clean:
	@ # Remove all .obj files
	@ rm *.obj 2> /dev/null || true