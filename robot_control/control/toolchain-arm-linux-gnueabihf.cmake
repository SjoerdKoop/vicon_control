# CMake toolchain file for building ARM software on OI environment

# This one is important
SET(CMAKE_SYSTEM_NAME Linux)

# This one not so much
SET(CMAKE_SYSTEM_VERSION 1)

# Specify the cross compiler
SET(CMAKE_C_COMPILER   /usr/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
SET(CMAKE_STRIP /usr/bin/arm-linux-gnueabihf-strip)

# Where is the target environment 
SET(CMAKE_FIND_ROOT_PATH  /usr/arm-linux-gnueabihf)

# Search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# For libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)