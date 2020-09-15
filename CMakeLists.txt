cmake_minimum_required(VERSION 2.8.12)
project(multi_lidar_merger)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

# MultiLidar Calibrator
add_library(multi_lidar_merger_lib SHARED
  src/multi_lidar_merger.cpp
)

target_include_directories(multi_lidar_merger_lib PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(multi_lidar_merger_lib
  ${catkin_LIBRARIES}
)

add_executable(multi_lidar_merger
  src/multi_lidar_merger.cpp
)

target_link_libraries(multi_lidar_merger
  multi_lidar_merger_lib
)

install(
  TARGETS
    multi_lidar_merger
    multi_lidar_merger_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
