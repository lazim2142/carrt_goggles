cmake_minimum_required(VERSION 2.8.3)
project(carrt_goggles)

set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rospy tf tf_conversions pcl_ros sensor_msgs cv_bridge image_transport dynamic_reconfigure)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(cfg/Disparity.cfg)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(obstacle_detector src/obstacle_detector.cpp
                include/collision_detector.h src/collision_detector.cpp)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ground_plane src/ground_plane.cpp)
target_link_libraries(ground_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(fisheye_rect src/fisheye_rect.cpp)
add_dependencies(fisheye_rect ${PROJECT_NAME}_gencfg)
target_link_libraries(fisheye_rect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(audio_buffer_node src/audio_buffer_main.cpp)
target_link_libraries(audio_buffer_node ${catkin_LIBRARIES})

add_executable(face_rec src/face_rec.cpp)
target_link_libraries(face_rec ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
