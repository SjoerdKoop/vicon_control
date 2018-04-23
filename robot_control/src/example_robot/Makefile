################################################################################
########## DO NOT EDIT THE SECTION UNLESS YOU KNOW WHAT YOU ARE DOING ##########
# Binary and library destination paths
BIN_PATH = ../../bin
LIB_PATH = ../../lib

# ARM binaries
CXX = /usr/bin/arm-linux-gnueabihf-g++	# Compiler binary
AR = /usr/bin/arm-linux-gnueabihf-ar	# Archiver binary

# Set compiler flags
FLAGS = -std=c++11 $(INCLUDE_DIRS:%=-I%) -L$(LIB_PATH)
################################################################################

# Include directories, add all directories of used header files
INCLUDE_DIRS = include ../components/include ../robot/include ../tools/include

# Source files: add source files of targets
# <NAME>_SRCS = src/<src1>.cpp src/<src2>.cpp ...
EXAMPLE_SRCS = src/example_robot.cpp src/example_robot_controller.cpp

# Library files: add library files of targets
# For include libfile1, use -lfile1
# <NAME>_LIBS = -lfile1 -lfile2 ...
EXAMPLE_LIBS = -lrobot -ltools -lcomponents -lpthread

# Default target
default: create clean

# Create target
create:
	@ # Creates the directories if they do not exist
	@ [ -d $(BIN_PATH) ] || mkdir -p $(BIN_PATH)
	@ [ -d $(LIB_PATH) ] || mkdir -p $(LIB_PATH)

	@ # Invoke the ARM compiler to create binaries and libraries:

	@ # For binaries to BIN_PATH:
	@ # $(CXX) $(FLAGS) $(<name>_SRCS) $(<name>_LIBS) -o $(BIN_PATH)/<name>

	@ # For libraries to LIB_PATH:
	@ # $(CXX) -c $(FLAGS) $(<name>_SRCS) -o <name>.o
	@ # $(AR) crf $(LIB_PATH)/lib<name>.a -o <name>.o

	@ # Create example_robot binary
	@ $(CXX) $(FLAGS) $(EXAMPLE_SRCS) $(EXAMPLE_LIBS) -o $(BIN_PATH)/example_robot

# Clean target
clean:
	@ # Remove all .o files
	@ rm *.o 2> /dev/null || true