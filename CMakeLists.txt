################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.10.1)
project(hexapod_controller)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -lrt")

################################################################################
# Find packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cmake_modules
)
find_package(Eigen3 REQUIRED)

find_package(jsoncpp REQUIRED)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  LIBRARIES hexapod_robot
)

################################################################################
# Build
################################################################################

set(SOURCE_FILES
    src/robot.cpp
    src/leg.cpp
  )

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#add_library(robot ${SOURCE_FILES})
add_library(hexapod_robot STATIC ${SOURCE_FILES})

## Declare a C++ executable
add_executable(hexapod_controller src/main.cpp)

## Add cmake target dependencies of the executable

## Specify libraries to link a library or executable target against
target_link_libraries(hexapod_controller
	hexapod_robot
	${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  jsoncpp
	)



## Add cmake target dependencies of the executable
#add_dependencies(dynamixel_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(dynamixel_sdk ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS hexapod_robot hexapod_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
