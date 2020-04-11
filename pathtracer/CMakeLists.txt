
cmake_minimum_required(VERSION 3.2.1)
project(ecobot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
  nav_core 
  move_base_msgs
  base_local_planner
  roscpp
  std_msgs
  tf
  rostest
  roscpp
  rospy
  message_generation)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cleaner_robot
   CATKIN_DEPENDS nav_core roscpp std_msgs
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(pathPlanner src/gridSquare.cpp src/pathPlanner.cpp)

add_executable(tracer src/randomizer.cpp src/spawnCollect.cpp src/collector.cpp)
target_link_libraries(tracer ${catkin_LIBRARIES})

add_executable(data src/main.cpp src/navigateRobot.cpp)
add_dependencies(data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(data ${catkin_LIBRARIES})

