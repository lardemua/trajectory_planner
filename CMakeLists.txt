cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  interactive_markers
  pcl_ros
  pcl_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  coordinates.msg
  traj_info.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES trajectory_planner
 CATKIN_DEPENDS roscpp std_msgs message_runtime interactive_markers pcl_ros pcl_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} include)

add_library(trajectory src/c_trajectory.cpp src/c_manage_trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})


add_executable(trajectory_planner_nodelet src/trajectory_planner_nodelet.cpp)
add_executable(APgenerator src/APgenerator.cpp)
add_executable(APwaypoints src/APwaypoints.cpp)
add_executable(mtt_target_generator src/mtt_target_generator.cpp)
add_executable(tf_generator src/tf_generator.cpp)
add_executable(tf_generator2 src/tf_generator2.cpp)
add_executable(tracking_frame_generator src/tracking_frame_generator.cpp)
add_executable(trajectory_executive src/trajectory_executive.cpp)

add_dependencies(trajectory trajectory_planner_generate_messages_cpp)
add_dependencies(trajectory_planner_nodelet trajectory_planner_generate_messages_cpp)
add_dependencies(APgenerator trajectory_planner_generate_messages_cpp)
add_dependencies(APwaypoints trajectory_planner_generate_messages_cpp)
add_dependencies(mtt_target_generator trajectory_planner_generate_messages_cpp)
add_dependencies(tf_generator trajectory_planner_generate_messages_cpp)
add_dependencies(tf_generator2 trajectory_planner_generate_messages_cpp)
add_dependencies(tracking_frame_generator trajectory_planner_generate_messages_cpp)
add_dependencies(trajectory_executive trajectory_planner_generate_messages_cpp)

target_link_libraries(trajectory_planner_nodelet trajectory ${catkin_LIBRARIES})
target_link_libraries(APgenerator  ${catkin_LIBRARIES})
target_link_libraries(APwaypoints  ${catkin_LIBRARIES})
target_link_libraries(mtt_target_generator  ${catkin_LIBRARIES})
target_link_libraries(tf_generator ${catkin_LIBRARIES})
target_link_libraries(tf_generator2 ${catkin_LIBRARIES})
target_link_libraries(tracking_frame_generator ${catkin_LIBRARIES})
target_link_libraries(trajectory_executive ${catkin_LIBRARIES})

## Declare a cpp library
# add_library(trajectory_planner
#   src/${PROJECT_NAME}/trajectory_planner.cpp
# )

## Declare a cpp executable
# add_executable(trajectory_planner_node src/trajectory_planner_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(trajectory_planner_node trajectory_planner_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(trajectory_planner_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trajectory_planner trajectory_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
