# RPi4 Cross Compilation Toolchain File
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Use the default compilers on the Pi
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

# Specify the path to the RPLIDAR SDK
set(RPLIDAR_SDK_DIR ${CMAKE_SOURCE_DIR}/include/rplidar_sdk) # Update this path to the location of your RPLIDAR SDK
set(RPLIDAR_SDK_LIB_DIR ${RPLIDAR_SDK_DIR}/sdk)

