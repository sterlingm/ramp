cmake_minimum_required(VERSION 2.8.3)
project(ramp_sensing)

find_package(catkin REQUIRED COMPONENTS geometry_msgs map_msgs message_generation nav_msgs ramp_msgs roscpp tf)
find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

#######################################
## Declare ROS messages and services ##
#######################################


catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp ramp_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

## Debugging flag for using gdb
set (CMAKE_CXX_FLAGS "-std=c++11 -g -O0")


add_executable(${PROJECT_NAME} src/circle_filter.cpp src/circle_packer.cpp src/GridMap2D.cpp src/main.cpp src/obstacle.cpp src/utility.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${BFL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ramp_msgs_generate_messages_cpp)

add_executable(convert_hilbert_data src/main_convert_map_to_hilbert_data.cpp)
target_link_libraries(convert_hilbert_data ${catkin_LIBRARIES} ${OpenCV_LIBS} ${BFL_LIBRARIES})
add_dependencies(convert_hilbert_data ramp_msgs_generate_messages_cpp)

add_executable(process_hmap src/main_process_hmap.cpp src/circle_packer.cpp src/GridMap2D.cpp src/obstacle.cpp src/utility.cpp)
target_link_libraries(process_hmap ${catkin_LIBRARIES} ${OpenCV_LIBS} ${BFL_LIBRARIES})
add_dependencies(process_hmap ramp_msgs_generate_messages_cpp)

add_executable(pub_laser_tf src/main_laser_frame.cpp)
target_link_libraries(pub_laser_tf ${catkin_LIBRARIES})

add_executable(pub_tf_costmap src/main_map_rotate.cpp)
target_link_libraries(pub_tf_costmap ${catkin_LIBRARIES})

add_executable(pub_map_equal_odom src/main_map_equal_odom.cpp)
target_link_libraries(pub_map_equal_odom ${catkin_LIBRARIES})

add_executable(clear_obs src/main_clearing.cpp)
target_link_libraries(clear_obs ${catkin_LIBRARIES})


add_executable(pub_bf_bl src/main_tf_basefp_baselink.cpp)
target_link_libraries(pub_bf_bl ${catkin_LIBRARIES})


add_executable(pub_tf_missing_turtlebot_gazebo src/main_pub_tf_missing_turtlebot_gazebo.cpp)
target_link_libraries(pub_tf_missing_turtlebot_gazebo ${catkin_LIBRARIES})
