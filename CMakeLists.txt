cmake_minimum_required(VERSION 2.8.3)
project(carrt_goggles)

## C++11
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rospy tf tf_conversions PCL pcl_ros cmake_modules)

catkin_package(
    DEPENDS Eigen
    )

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
add_executable(obstacle_detector src/obstacle_detector.cpp)
add_executable(ground_plane src/ground_plane.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(ground_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES})
