cmake_minimum_required(VERSION 3.0.2)
project(tamiya_jetracer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  urdf
  tf
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  vesc_feedback.msg
  vesc_cmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy
  urdf
  tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(odom_publisher
    src/odom_publisher.cpp
)

add_dependencies(odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(odom_publisher
    ${catkin_LIBRARIES}
)
