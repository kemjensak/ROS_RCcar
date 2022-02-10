cmake_minimum_required(VERSION 2.8.3)
project(stanley)

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    geometry_msgs
    visualization_msgs
    std_msgs
    tf
    libwaypoint_follower
    roscpp
    roslint
    rostest
    rosunit
)

catkin_package(
  INCLUDE_DIRS include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  stanley
  src/stanley_node.cpp
  src/stanley.cpp
  src/stanley_core.cpp
  src/stanley_viz.cpp
)
target_link_libraries(stanley ${catkin_LIBRARIES})
add_dependencies(stanley ${catkin_EXPORTED_TARGETS})

install(
  TARGETS stanley
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   add_rostest_gtest(
#     test-stanley
#     test/test_stanley.test
#     test/src/test_stanley.cpp
#     src/stanley_core.cpp
#     src/stanley.cpp src/stanley_viz.cpp
#   )
#   add_dependencies(test-stanley ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(test-stanley ${catkin_LIBRARIES})

#   roslint_add_test()
# endif()
