cmake_minimum_required(VERSION 2.8.3)
project(sync_pointcloud)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast -DEIGEN_DONT_PARALLELIZE -pthread ${OpenMP_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_geometry
  message_filters
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)

catkin_package(INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet
  src/nodelets/sync_pointcloud_nodelet.cpp
  src/sync_pointcloud.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})


add_executable(sync_pointcloud_node src/nodes/sync_pointcloud_node.cpp)
target_link_libraries(sync_pointcloud_node ${catkin_LIBRARIES})

# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)