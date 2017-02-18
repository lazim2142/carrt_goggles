cmake_minimum_required(VERSION 2.8.3)
project(carrt_goggles)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(carrt_goggles
#   src/${PROJECT_NAME}/carrt_goggles.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(carrt_goggles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(obstacle_detector src/obstacle_detector.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(obstacle_detector ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(carrt_goggles_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


