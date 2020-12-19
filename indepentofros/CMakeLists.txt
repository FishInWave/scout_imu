cmake_minimum_required(VERSION 2.8.3)
project(imu)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE "Debug")
add_executable(imu
	src/main.cpp 
	src/imu.cpp 
)

include_directories(
  include
)
