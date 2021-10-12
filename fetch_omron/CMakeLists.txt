cmake_minimum_required(VERSION 3.7.2)
project(fetch_omron)

find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    angles
    control_toolbox
    gazebo_ros
    robot_controllers
    robot_controllers_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    gazebo_plugins
    gazebo_ros
    robot_controllers
    robot_controllers_interface
  LIBRARIES
    fetch_omron_plugin
  INCLUDE_DIRS
    include
)

add_library(fetch_omron_plugin src/fetch_plugin.cpp)
target_link_libraries(fetch_omron_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS fetch_omron_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  PROGRAMS
    scripts/prepare_simulated_robot.py
    scripts/prepare_simulated_robot_pick_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config include launch robots worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
