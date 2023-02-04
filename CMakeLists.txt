cmake_minimum_required( VERSION 3.0 )

project( Voxel_Carving )

find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

include_directories( ${OpenCV_INCLUDE_DIRS} include )
file( GLOB SRC "src/*.cpp" )
# add_executable( ${PROJECT_NAME} ${SRC} )
# target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

add_executable(create_aruco_board src/create_aruco_board.cpp)
target_link_libraries( create_aruco_board ${OpenCV_LIBS} )
add_executable(pose_estimation_aruco_board src/pose_estimation_aruco_board.cpp)
target_link_libraries( pose_estimation_aruco_board ${OpenCV_LIBS} )
add_executable( camera_calibration src/camera_calibration.cpp )
target_link_libraries( camera_calibration ${OpenCV_LIBS} )
add_executable( background_subtraction src/backgroundSub.cpp )
target_link_libraries( background_subtraction ${OpenCV_LIBS} )
add_executable( pose_estimation_aruco_board_image_list src/pose_estimation_aruco_board_image_list.cpp )
target_link_libraries( pose_estimation_aruco_board_image_list ${OpenCV_LIBS} )
add_executable(main src/main.cpp src/Volume.cpp src/VoxelCarving.cpp)
target_link_libraries(main ${OpenCV_LIBS} Eigen3::Eigen)


