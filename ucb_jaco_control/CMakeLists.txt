cmake_minimum_required(VERSION 2.8.3)
project(ucb_jaco_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
  controller_interface
  dynamic_reconfigure
  kdl_parser
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
find_package(PythonLibs 2.7)
include_directories(${PYTHON_INCLUDE_DIRS})

set(UCB_JACO_CONTROL_ROS ucb_jaco_control_ros)
set(UCB_JACO_CONTROL ucb_jaco_control)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

generate_dynamic_reconfigure_options(
  config/PIDGains.cfg
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
  LIBRARIES ${UCB_JACO_CONTROL_ROS} ${UCB_JACO_CONTROL}
  CATKIN_DEPENDS controller_interface
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ucb_jaco_control.cpp
# )

# Library of ROS control plugins.
add_library(${UCB_JACO_CONTROL_ROS}
  src/test_controller_ros.cpp
)
target_link_libraries(${UCB_JACO_CONTROL_ROS} ${catkin_LIBRARIES})
add_dependencies(${UCB_JACO_CONTROL_ROS} ${PROJECT_NAME}_gencfg)

add_executable(jaco_simulator_ros src/jaco_simulator_ros.cpp)
target_link_libraries(jaco_simulator_ros ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

# Library containing core controllers.
#add_library(${UCB_JACO_CONTROL} INTERFACE)
#target_include_directories(${UCB_JACO_CONTROL} INTERFACE include/${UCB_JACO_CONROL})
#target_link_libraries(${UCB_JACO_CONTROL} ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ucb_jaco_control_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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
install(TARGETS ${UCB_JACO_CONTROL_ROS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES ${UCB_JACO_CONTROL_ROS}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ucb_jaco_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

catkin_add_gtest(pid_regulation_controller_test test/test_pid_regulation_controller.cpp)
if (TARGET pid_regulation_controller_test)
  #target_link_libraries(pid_regulation_controller_test ${UCB_JACO_CONTROL})

  #target_include_directories(pid_regulation_controller_test PRIVATE ${PYTHON_INCLUDE_DIRS})
  target_link_libraries(pid_regulation_controller_test ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
endif()

catkin_add_gtest(constant_trajectory_test test/test_constant_trajectory.cpp)
if (TARGET constant_trajectory_test)
  target_link_libraries(constant_trajectory_test ${catkin_LIBRARIES})
endif()

catkin_add_gtest(sinusoidal_trajectory_test test/test_sinusoidal_trajectory.cpp)
if (TARGET sinusoidal_trajectory_test)
  set(CMAKE_BUILD_TYPE Debug)
  target_link_libraries(sinusoidal_trajectory_test ${catkin_LIBRARIES})
endif()

catkin_add_gtest(robot_dynamics_test test/test_robot_dynamics.cpp)
if (TARGET robot_dynamics_test)
  target_link_libraries(robot_dynamics_test ${catkin_LIBRARIES})
endif()

catkin_add_gtest(buffer_test test/test_buffer.cpp)
if (TARGET buffer_test)
  set(CMAKE_BUILD_TYPE Debug)
  target_link_libraries(buffer_test ${catkin_LIBRARIES})
endif()

catkin_add_gtest(runge_kutta_test test/test_runge_kutta.cpp)
if (TARGET runge_kutta_test)
  set(CMAKE_BUILD_TYPE Debug)
  target_link_libraries(runge_kutta_test ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
endif()

catkin_add_gtest(pid_computed_torque_control_test test/test_pid_computed_torque_control.cpp)
if (TARGET pid_computed_torque_control_test)
  set(CMAKE_BUILD_TYPE Debug)
  target_link_libraries(pid_computed_torque_control_test ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
endif()
