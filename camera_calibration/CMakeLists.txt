cmake_minimum_required(VERSION 2.8)

set (PROJECTNAME camera_calibration)

project( ${PROJECTNAME} )

#set (OpenCV_LIBS
#	/usr/local/lib/libopencv_core.so
#	/usr/local/lib/libopencv_highgui.so
#	/usr/local/lib/libopencv_imgproc.so)

find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set (CMAKE_BUILD_TYPE Debug)
add_executable( ${PROJECTNAME} camera_calibration.cpp )
target_link_libraries( ${PROJECTNAME} ${OpenCV_LIBS} )
