cmake_minimum_required(VERSION 3.0.2)
project(cricball)
## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
	gazebo_ros 
	roscpp 
	rospy
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For tc_simple_world_plugin plugin
add_library(happy_world_plugin1 src/simple_world_plugin.cpp)
target_link_libraries(happy_world_plugin1 ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For model_push_plugin plugin
#add_library(model_push_plugin src/model_push.cc)
#target_link_libraries(model_push_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For earthquake_plugin plugin
#add_library(earthquake_plugin src/earthquake.cc)
#target_link_libraries(earthquake_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
  	gazebo_ros 
    roscpp 
    rospy
)

catkin_install_python(
  PROGRAMS
  scripts/ball_detection.py
  scripts/ball_detection1.py
  scripts/trajectory_lsr.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
