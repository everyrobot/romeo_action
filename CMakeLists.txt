cmake_minimum_required(VERSION 2.8.3)
SET(PROJECTNAME romeo_pick_place_adv)
project(${PROJECTNAME})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  #moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_simple_grasps
  std_msgs
  geometry_msgs
  roscpp
  cmake_modules
  #object_recognition_msgs
  #moveit_planning_scene_interface
)

#find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)

#find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

# Enable gdb to show line numbers
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
#set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(SRC
  src/block_pick_place.cpp
  src/metablock.cpp
  src/actions.cpp
  include/romeo_pick_place/simplepickplace.hpp
  include/romeo_pick_place/metablock.hpp
  include/romeo_pick_place/actions.hpp
  include/romeo_pick_place/custom_environment5.hpp
)

# Main executable
add_executable(${PROJECTNAME} ${SRC})
target_link_libraries(${PROJECTNAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

