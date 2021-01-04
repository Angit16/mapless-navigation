#Version of CMake required
cmake_minimum_required(VERSION 2.8.3)

#Name of this package
project(blind_planner)

# Find catkin build system, and any other packages on which we depend
find_package(catkin REQUIRED COMPONENTS
 roscpp
 geometry_msgs
 nav_msgs
 std_msgs
 tf
 tf2
 tf2_ros
 actionlib
 actionlib_msgs)

add_action_files(
  DIRECTORY action
  FILES GoToPose.action
)

generate_messages (
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

#Declare catkin package

include_directories( include  ${catkin_INCLUDE_DIRS})
catkin_package(
INCLUDE_DIRS include)
# Specify locations of header files


#Delare the executable, along with its source files.
add_executable(test_Client src/test_Client.cpp)
add_dependencies(test_Client ${PROJECT_NAME}_EXPORTED_TARGETS)
target_link_libraries (test_Client ${catkin_LIBRARIES})
add_executable(test_server src/test_server.cpp)
add_dependencies(test_server ${PROJECT_NAME}_EXPORTED_TARGETS)
#Specify libraries against which to link.
target_link_libraries (test_server ${catkin_LIBRARIES})


catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
