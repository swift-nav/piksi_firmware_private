# We require at least 3.6 to have support for the CMAKE_TRY_COMPILE_TARGET_TYPE variable
cmake_minimum_required(VERSION 3.6)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR arm)
# This needs to be set, but we don't version our "system"
SET(CMAKE_SYSTEM_VERSION 1)

# Specify the cross compiler
# Note: This searches the system for any compiler with this name, which
# might grab something we don't want.
SET(CMAKE_C_COMPILER   arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# This tells CMake to compile test code into a static library
# since we aren't providing full details on how to link executeables
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Set some base compile flags
set(CMAKE_C_FLAGS   "-fno-common -ffunction-sections -fdata-sections -mcpu=cortex-a9 -march=armv7-a -mthumb -mfloat-abi=hard -mfpu=neon" CACHE STRING "")
set(CMAKE_CXX_FLAGS "-fno-common -ffunction-sections -fdata-sections -mcpu=cortex-a9 -march=armv7-a -mthumb -mfloat-abi=hard -mfpu=neon" CACHE STRING "")

# Tell CMake to build static libs by default
set(BUILD_SHARED_LIBS CACHE BOOL OFF FORCE)
