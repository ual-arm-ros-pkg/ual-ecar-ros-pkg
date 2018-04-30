
CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)  # Require C++14
project(battery_charge)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	message_generation
  roscpp
  std_msgs
)


# System dependencies are found with CMake's conventions
find_package(MRPT 1.5 REQUIRED)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
if (MRPT_VERSION VERSION_LESS 1.9.9)
	find_package(MRPT REQUIRED hwdrivers)
else()
	find_package(MRPT REQUIRED comms)
endif()


################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  DIRECTORY msg
  FILES
	BatReading.msg
	AnalogReading.msg
)

generate_messages(DEPENDENCIES std_msgs)

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
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${${PROJECT_NAME}_SOURCE_DIR}/firmware/battery-firmware/firmware/firmware/ # Path del .h comun de la estructura de datos
  ${${PROJECT_NAME}_SOURCE_DIR}/include
)


# Declare a C++ library
file(GLOB lib_srcs  src/${PROJECT_NAME}/*.cpp include/${PROJECT_NAME}/*.h)

add_library(${PROJECT_NAME}
   ${lib_srcs}
)

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${MRPT_LIBS}
   )

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   ${MRPT_LIBS}
)

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

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_steer_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
