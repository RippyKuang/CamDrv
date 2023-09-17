cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -Wall -g -ggdb")
add_definitions(-w)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
project (CamDrv
	VERSION 0.1
	DESCRIPTION "A Camera driver Project")
find_package(OpenCV REQUIRED)
add_executable(${PROJECT_NAME} CamDrv.cpp)

configure_file(config.h.in config.h)

target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}")
target_link_libraries(${PROJECT_NAME}
					 ${OpenCV_LIBS}
                      /lib/libMVSDK.so)