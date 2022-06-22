################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(jinu_manipulator_controller)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  jinu_manipulator_libs
  jinu_manipulator_msgs
  robotis_manipulator
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Core REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jinu_manipulator_controller
  CATKIN_DEPENDS 
    cmake_modules 
    geometry_msgs 
    jinu_manipulator_libs 
    jinu_manipulator_msgs 
    robotis_manipulator 
    roscpp 
    sensor_msgs 
    std_msgs 
    trajectory_msgs
  DEPENDS 
    Boost
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#add_library(rmd_motor src/rmd_motor.cpp)

add_executable(
  jinu_manipulator_controller 
  src/jinu_manipulator_controller.cpp 
  src/rmd_utils.cpp 
  src/rmd_can.cpp
  src/spi2can.cpp
  src/rmd_motor.cpp)
add_dependencies(jinu_manipulator_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(jinu_manipulator_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} Qt5::Core)

################################################################################
# Install
################################################################################
install(TARGETS jinu_manipulator_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/update_spi_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################