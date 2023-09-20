include_directories("${PROJECT_BINARY_DIR}"
		    "${CMAKE_SOURCE_DIR}/inc"
		    )
find_package(OpenCV REQUIRED)
add_library(DRV CamDrv.cpp)
target_link_libraries(DRV
					 ${OpenCV_LIBS}
                      /lib/libMVSDK.so
                      )