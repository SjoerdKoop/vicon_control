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
INCLUDE_DIRS = include ../components/include

# Source files: add source files of targets
# <NAME>_SRCS = src/<src1>.cpp src/<src2>.cpp ...
TOOLS_SRCS = src/tools.cpp
DATA_SERVER_SRCS = src/data_server.cpp
READ_SHARED_MEMORY_SRCS = src/read_shared_memory.cpp
REFERENCE_CLIENT_SRCS = src/reference_client.cpp
WRITE_SHARED_MEMORY_SRCS = src/write_shared_memory.cpp

# Library files: add library files of targets
# For include libfile1, use -lfile1
# <NAME>_LIBS = -lfile1 -lfile2 ...
DATA_SERVER_LIBS = -lcomponents -ltools
READ_SHARED_MEMORY_LIBS = -lcomponents -ltools
REFERENCE_CLIENT_LIBS = -lcomponents -ltools
WRITE_SHARED_MEMORY_LIBS = -lcomponents -ltools

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
	@ # $(CXX) -c $(FLAGS) $(<name>_SRCS) $(<name>_LIBS) -o <name>.o
	@ # $(AR) crf $(LIB_PATH)/lib<name>.a -o <name>.o

	@ # Create tools library
	@ $(CXX) -c $(FLAGS) $(TOOLS_SRCS) -o tools.o
	@ $(AR) crf $(LIB_PATH)/libtools.a -o tools.o

	@ # Create data_server binary
	@ $(CXX) $(FLAGS) $(DATA_SERVER_SRCS) $(DATA_SERVER_LIBS)\
		-o $(BIN_PATH)/data_server

	@ # Create read_shared_memory binary
	@ $(CXX) $(FLAGS) $(READ_SHARED_MEMORY_SRCS) $(READ_SHARED_MEMORY_LIBS)\
		-o $(BIN_PATH)/read_shared_memory

	@ # Create reference_client binary
	@ $(CXX) $(FLAGS) $(REFERENCE_CLIENT_SRCS) $(REFERENCE_CLIENT_LIBS)\
		-o $(BIN_PATH)/reference_client

	@ # Create write_shared_memory binary
	@ $(CXX) $(FLAGS) $(WRITE_SHARED_MEMORY_SRCS) $(WRITE_SHARED_MEMORY_LIBS)\
		-o $(BIN_PATH)/write_shared_memory

# Clean target
clean:
	@ # Remove all .o files
	@ rm *.o 2> /dev/null || true