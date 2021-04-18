cmake_minimum_required(VERSION 3.0.2)
project(scan_matching)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf
  geometry_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES point transform poly_root correspondence
  CATKIN_DEPENDS nav_msgs roscpp std_msgs sensor_msgs geometry_msgs tf 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(scan_matcher src/scan_match.cpp src/point.cpp src/transform.cpp src/correspondence.cpp src/poly_root.cpp)
target_link_libraries(scan_matcher ${catkin_LIBRARIES})