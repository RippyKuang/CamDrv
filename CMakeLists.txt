cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
project (CamDrv
	VERSION 1.2
	DESCRIPTION "A Camera driver Project")
add_executable(${PROJECT_NAME}  "main.cpp" "${CMAKE_SOURCE_DIR}/armor/armor.cpp")
add_subdirectory(drv)
configure_file(config.h.in config.h)
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}" "${CMAKE_SOURCE_DIR}/inc"
	)
target_link_libraries(${PROJECT_NAME} DRV)
