cmake_minimum_required(VERSION 3.0)

project(${FOO_PROJECT_NAME})

project( Voxel_Carving )

find_package( OpenCV REQUIRED )
set(OpenCV_LIB_PATH $ENV{OpenCV_DIR}/x64/mingw/lib)
include("${OpenCV_LIB_PATH}/OpenCVConfig.cmake")

include_directories( ${OpenCV_INCLUDE_DIRS} include)
file(GLOB SRC "src/*.cpp")
# add_executable( ${PROJECT_NAME} ${SRC} )
# target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

add_executable(create_aruco_border src/create_aruco_border.cpp)
target_link_libraries( create_aruco_border ${OpenCV_LIBS} )
add_executable(pose_estimation_aruco_border src/pose_estimation_aruco_border.cpp)
target_link_libraries( pose_estimation_aruco_border ${OpenCV_LIBS} )
add_executable( camera_calibration src/camera_calibration.cpp )
target_link_libraries( camera_calibration ${OpenCV_LIBS} )


