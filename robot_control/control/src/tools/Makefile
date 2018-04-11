### Project ###
# Only edit this section of the file

# Include directories, add all directories of used header files
INCLUDE_DIRS = include ../components/include

# Source files: add all your source files
SOURCE_FILES = src/reference_client.cpp src/tools.cpp ../components/src/peer.cpp

# Set firmware name, will be found in FIRMWARE_PATH
BIN_NAME = reference_client

### Template ###
# DO NOT EDIT UNLESS YOU KNOW WHAT YOU ARE DOING

# Binary destination path
BIN_PATH = ../../bin

# Library path
LIB_PATH = ../../lib

# Global files
ARM_COMPILER_BIN = /usr/bin/arm-linux-gnueabihf-g++		# ARM compiler binary

# Flags
COMPILER_FLAGS = -std=c++11 $(INCLUDE_DIRS:%=-I%) -L$(LIB_PATH)


# Default target
default:
	@export $(shell TEST_VAL=3)

	@echo $(TEST_VAL2)
	@ # Creates the directory if it does not exists already
	@ [ -d $(BIN_PATH) ] || mkdir -p $(BIN_PATH)

	@ # Invoke the ARM compiler
	@ $(ARM_COMPILER_BIN) $(COMPILER_FLAGS) $(SOURCE_FILES) -o $(BIN_PATH)/$(BIN_NAME)