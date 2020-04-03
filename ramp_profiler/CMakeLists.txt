cmake_minimum_required(VERSION 2.8.3)
project(ramp_profiler)
#
find_package(catkin REQUIRED COMPONENTS message_generation ramp_msgs roscpp roslib swri_profiler swri_profiler_msgs)
#
#
########################################
### Declare ROS messages and services ##
########################################
#
#
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS 
)


############
### Build ##
############

### Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

### Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-g")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_executable(profiler src/profiler.cpp)
target_link_libraries(profiler ${catkin_LIBRARIES})
add_dependencies(profiler ramp_msgs_generate_messages_cpp)