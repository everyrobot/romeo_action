cmake_minimum_required(VERSION 2.8.3)
#SET(PROJECTNAME romeo_pick_place_adv)
SET(PROJECTNAME pick_place_moveit)
project(${PROJECTNAME})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_simple_grasps
  std_msgs
  geometry_msgs
  roscpp
  cmake_modules
  object_recognition_msgs
  actionlib
)

#find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread date_time system filesystem)
find_package(Boost QUIET COMPONENTS program_options)

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

link_directories(${Boost_LIBRARY_DIRS})

set(SRC
  src/main.cpp
  src/simplepickplace.cpp
  src/metablock.cpp
  src/action.cpp
  src/postures.cpp
  src/objprocessing.cpp
  src/actionclient.cpp
  src/grasp_filter.cpp
  include/romeo_pick_place/simplepickplace.hpp
  include/romeo_pick_place/metablock.hpp
  include/romeo_pick_place/action.hpp
  include/romeo_pick_place/custom_environment5.hpp
  include/romeo_pick_place/postures.hpp
  include/romeo_pick_place/tools.hpp
  include/romeo_pick_place/toolsForAction.hpp
  include/romeo_pick_place/objprocessing.hpp
  include/romeo_pick_place/actionclient.hpp
  include/romeo_pick_place/grasp_filter.hpp
)

# Main executable
add_executable(${PROJECTNAME} ${SRC})

target_link_libraries(${PROJECTNAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

