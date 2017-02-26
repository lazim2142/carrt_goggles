cmake_minimum_required(VERSION 2.8.3)
project(carrt_goggles)

## C++11
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp rospy)
find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
add_executable(obstacle_detector src/obstacle_detector.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
