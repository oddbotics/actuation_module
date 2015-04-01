cmake_minimum_required(VERSION 2.8.3)
project(actuation_module)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  oddbot_msgs
  roscpp
  sensor_msgs
  std_msgs
  beaglebone_eqep
  beaglebone_blacklib
  tf
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(actuation_module_lib
  src/${PROJECT_NAME}/PID.cpp
)

## Declare a cpp executable
add_executable(beaglebone_dc_motor_controller src/beaglebone_dc_motor_controller.cpp
	src/dc_motor_controller.cpp
	src/position_controller.cpp
	src/velocity_controller.cpp
)

add_executable(beaglebone_dc_motor_tf src/beaglebone_dc_motor_tf.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(actuation_module_node actuation_module_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(beaglebone_dc_motor_controller
  ${catkin_LIBRARIES}
  actuation_module_lib
  beaglebone_blacklib
  beaglebone_eqep
)

target_link_libraries(beaglebone_dc_motor_tf
  ${catkin_LIBRARIES}
)

set_target_properties(beaglebone_dc_motor_controller PROPERTIES COMPILE_FLAGS -std=c++11)
set_target_properties(beaglebone_dc_motor_tf PROPERTIES COMPILE_FLAGS -std=c++11)
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
# install(TARGETS actuation_module actuation_module_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_actuation_module.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
