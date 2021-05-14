cmake_minimum_required(VERSION 3.0.2)
project(tamiya_jetracer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  urdf
  tf
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



add_executable(${PROJECT_NAME}
    src/odom_publisher.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
