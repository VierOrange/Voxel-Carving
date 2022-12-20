cmake_minimum_required(VERSION 2.8)

project( Voxel_Carving )

set(OpenCV_LIB_PATH $ENV{OpenCV_DIR}/x64/mingw/lib)
include("${OpenCV_LIB_PATH}/OpenCVConfig.cmake")
#find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( Voxel_Carving main.cpp calibration.cpp)
target_link_libraries( Voxel_Carving ${OpenCV_LIBS} )