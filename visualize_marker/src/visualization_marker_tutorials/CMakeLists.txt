cmake_minimum_required(VERSION 3.0.2)
project(visualization_marker_tutorials)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(yaml-cpp)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package()

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp sensor_msgs std_msgs visualization_msgs geometry_msgs
#   DEPENDS yaml-cpp
# )



###########
## Build ##
###########



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes
  ${catkin_LIBRARIES}
  yaml-cpp
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(TARGETS 
#   basic_shapes
#   # points_and_lines
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



