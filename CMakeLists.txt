cmake_minimum_required(VERSION 3.10)

project (CamDrv
	VERSION 0.1
	DESCRIPTION "A Camera driver Project")

add_executable(${PROJECT_NAME} CamDrv.cpp)

configure_file(config.h.in config.h)

target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}")
