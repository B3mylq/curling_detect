cmake_minimum_required(VERSION 2.8.3)
project(curling_detect)
 
add_compile_options(-std=gnu++11)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE -O2)
 
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  geometry_msgs
)
 
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depth_make kf_filter
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
)
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
 
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(kalman /home/b3mylq/ROS_ws/pcl_ws/src/curling_detect/include/kf_filter/kalman.cpp)
 
# add_executable(depth_make src/depth_make.cpp) 
# target_link_libraries(depth_make ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}) 
# add_dependencies(depth_make beginner_tutorials_generate_messages_cpp) 

# add_executable(stone_detection src/stone_detection.cpp) 
# target_link_libraries(stone_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}) 
# add_dependencies(stone_detection beginner_tutorials_generate_messages_cpp) 

# add_executable(euclidean_cluster src/euclidean_cluster.cpp) 
# target_link_libraries(euclidean_cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}) 
# add_dependencies(euclidean_cluster beginner_tutorials_generate_messages_cpp) 

# add_executable(echo_cluster src/echo_cluster.cpp) 
# target_link_libraries(echo_cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}) 
# add_dependencies(echo_cluster beginner_tutorials_generate_messages_cpp) 

add_executable(curling_detect src/curling_detect.cpp) 
target_link_libraries(curling_detect ${catkin_LIBRARIES} ${PCL_LIBRARIES}) 
add_dependencies(curling_detect beginner_tutorials_generate_messages_cpp) 

add_executable(kalman_test src/kalman_test.cpp) 
target_link_libraries(kalman_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} kalman) 
add_dependencies(kalman_test beginner_tutorials_generate_messages_cpp) 

add_executable(prompt_graph src/prompt_graph.cpp) 
target_link_libraries(prompt_graph ${catkin_LIBRARIES} ${PCL_LIBRARIES}) 
add_dependencies(prompt_graph beginner_tutorials_generate_messages_cpp)

add_executable(lidar_calibration src/lidar_calibration.cpp) 
target_link_libraries(lidar_calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES}) 
add_dependencies(lidar_calibration beginner_tutorials_generate_messages_cpp)

# add_executable(plane_segment src/plane_segment.cpp) 
# target_link_libraries(plane_segment ${catkin_LIBRARIES} ${PCL_LIBRARIES}) 
# add_dependencies(plane_segment beginner_tutorials_generate_messages_cpp) 
 
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse3 -std=c++14")

catkin_install_python(PROGRAMS
  scripts/udp_send_path.py
  scripts/udp_receive_path.py
  scripts/get_frame_id.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)