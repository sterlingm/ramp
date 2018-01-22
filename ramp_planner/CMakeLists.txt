cmake_minimum_required(VERSION 2.8.3)
project(ramp_planner)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs message_generation ramp_msgs roscpp roslib std_msgs tf)



########################################
### Declare ROS messages and services ##
########################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

############
### Build ##
############

# Specify include directories
include_directories(include ${catkin_INCLUDE_DIRS} /opt/ros/indigo/include)

## Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-g")

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/bezier_curve.cpp src/main.cpp src/planner.cpp src/control_handler.cpp src/knot_point.cpp src/modification_request_handler.cpp src/modifier.cpp src/motion_state.cpp src/parameter_handler.cpp src/path.cpp src/population.cpp src/ramp_trajectory.cpp src/range.cpp src/rviz_handler.cpp src/trajectory_request_handler.cpp src/evaluation_request_handler.cpp src/utility.cpp)

add_executable(ramp_planner_sipd src/bezier_curve.cpp src/main_sipd.cpp src/planner_sipd.cpp src/control_handler.cpp src/knot_point.cpp src/modification_request_handler.cpp src/modifier.cpp src/motion_state.cpp src/parameter_handler.cpp src/path.cpp src/population.cpp src/ramp_trajectory.cpp src/range.cpp src/rviz_handler.cpp src/trajectory_request_handler.cpp src/evaluation_request_handler.cpp src/utility.cpp)

# Add the -std argument to compile enum
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -std=c++0x)
set_target_properties(ramp_planner_sipd PROPERTIES COMPILE_FLAGS -std=c++0x)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(ramp_planner_sipd ${catkin_LIBRARIES} yaml-cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)
add_dependencies(ramp_planner_sipd ramp_msgs_generate_messages_cpp)

add_executable(pub_map_odom src/main_pub_map_odom.cpp)
target_link_libraries(pub_map_odom ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(obstacle_trajectory src/control_handler.cpp src/knot_point.cpp src/main_obstacle_trajectory.cpp src/motion_state.cpp src/path.cpp src/ramp_trajectory.cpp src/range.cpp src/trajectory_request_handler.cpp src/utility.cpp)

# Add the -std argument to compile enum
set_target_properties(obstacle_trajectory PROPERTIES COMPILE_FLAGS -std=c++0x)

## Specify libraries to link a library or executable target against
target_link_libraries(obstacle_trajectory ${catkin_LIBRARIES} yaml-cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(obstacle_trajectory ramp_msgs_generate_messages_cpp)


## Declare a cpp executable
add_executable(obstacle_twist_all src/main_obstacle_twist_all.cpp src/range.cpp src/utility.cpp)

# Add the -std argument to compile enum
set_target_properties(obstacle_twist_all PROPERTIES COMPILE_FLAGS -std=c++0x)

## Specify libraries to link a library or executable target against
target_link_libraries(obstacle_twist_all ${catkin_LIBRARIES} yaml-cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(obstacle_twist_all ramp_msgs_generate_messages_cpp)



# System-level testing executables
add_executable(run_test_case src/bezier_curve.cpp src/main_run_test_case.cpp src/planner.cpp src/control_handler.cpp src/knot_point.cpp src/modification_request_handler.cpp src/modifier.cpp src/motion_state.cpp src/parameter_handler.cpp src/path.cpp src/population.cpp src/ramp_trajectory.cpp src/range.cpp src/rviz_handler.cpp src/trajectory_request_handler.cpp src/evaluation_request_handler.cpp src/utility.cpp)
set_target_properties(run_test_case PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(run_test_case ${catkin_LIBRARIES} yaml-cpp)

add_executable(generate_test_case src/bezier_curve.cpp src/main_generate_test_case.cpp src/planner.cpp src/control_handler.cpp src/knot_point.cpp src/modification_request_handler.cpp src/modifier.cpp src/motion_state.cpp src/parameter_handler.cpp src/path.cpp src/population.cpp src/ramp_trajectory.cpp src/range.cpp src/rviz_handler.cpp src/trajectory_request_handler.cpp src/evaluation_request_handler.cpp src/utility.cpp)
set_target_properties(generate_test_case PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(generate_test_case ${catkin_LIBRARIES} yaml-cpp)


add_executable(move_gazebo_obs src/main_gazebo_ob.cpp src/range.cpp src/utility.cpp)
set_target_properties(move_gazebo_obs PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(move_gazebo_obs ${catkin_LIBRARIES} yaml-cpp)



## ============= Testing Section =============================================
catkin_add_gtest(fullPath_testFunctionality test/fullPath_testFunctionality.cpp src/range.cpp src/utility.cpp)
set_target_properties(fullPath_testFunctionality PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(fullPath_testFunctionality ${catkin_LIBRARIES} pthread)
##============================================================================
