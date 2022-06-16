cmake_minimum_required(VERSION 3.0.2)
project(jinu_manipulator_control)

# add_compile_options(-std=c++11)

FIND_PACKAGE(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  message_generation
)
generate_messages(DEPENDENCIES std_msgs)

find_package(gazebo REQUIRED)
find_package (Eigen3 REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS} ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} src/jm_simple.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 DEPENDS gazebo_ros roscpp rospy std_msgs
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")  
