

##  Copyright (c) 2019 Martim Brandão martim@robots.ox.ac.uk, Omer Burak Aladag aladagomer@sabanciuniv.edu, Ioannis Havoutis ioannis@robots.ox.ac.uk
##  This software is provided 'as-is', without any express or implied
##  warranty. In no event will the authors be held liable for any damages
##  arising from the use of this software.
##  Permission is granted to anyone to use this software for any purpose,
##  including commercial applications, and to alter it and redistribute it
##  freely, subject to the following restrictions:
##  1. The origin of this software must not be misrepresented; you must not
##    claim that you wrote the original software. If you use this software
##    in a product, an acknowledgment in the product documentation would be
##    appreciated but is not required.
##  2. Altered source versions must be plainly marked as such, and must not be
##    misrepresented as being the original software.
##  3. This notice may not be removed or altered from any source distribution.

cmake_minimum_required(VERSION 3.0.2)
project(recast_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  pcl_ros
  pcl_conversions
  pcl_msgs
  dynamic_reconfigure
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  message_generation
  interactive_markers
  recast_navigation
)

# message(FATAL_ERROR ${recast_navigation_INCLUDE_DIRS})

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake) 

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    RecastPathMsg.msg
    RecastGraphNode.msg
    RecastGraph.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   InputMeshSrv.srv
   RecastPathSrv.srv
   RecastProjectSrv.srv
   AddObstacleSrv.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   pcl_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/recast_node.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wabian_planner
  CATKIN_DEPENDS roscpp roslib visualization_msgs interactive_markers pcl_ros message_runtime geometry_msgs pcl_conversions pcl_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${recast_navigation_INCLUDE_DIRS}/recast_navigation  # to work in install-spaces
)

## recast test
file(GLOB RecastSources
  src/*.cpp
)

add_executable(recast_node src/nodes/recast_node.cpp ${RecastSources})
target_link_libraries(recast_node ${catkin_LIBRARIES})
add_dependencies(recast_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(recast_node  ${PROJECT_NAME}_gencfg)

add_executable(test_planning_service src/nodes/test_planning_service.cpp)
target_link_libraries(test_planning_service ${catkin_LIBRARIES})
add_dependencies(test_planning_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_add_obstacles src/nodes/test_add_obstacles.cpp)
target_link_libraries(test_add_obstacles ${catkin_LIBRARIES})
add_dependencies(test_add_obstacles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_remove_all_obstacles src/nodes/test_remove_all_obstacles.cpp)
target_link_libraries(test_remove_all_obstacles ${catkin_LIBRARIES})
add_dependencies(test_remove_all_obstacles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_planning_service_interactive src/nodes/test_planning_service_interactive.cpp)
target_link_libraries(test_planning_service_interactive ${catkin_LIBRARIES})
add_dependencies(test_planning_service_interactive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_input_map src/nodes/test_input_map.cpp ${RecastSources})
target_link_libraries(test_input_map ${catkin_LIBRARIES})
add_dependencies(test_input_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_update_parameters src/nodes/test_update_parameters.cpp ${RecastSources})
target_link_libraries(test_update_parameters ${catkin_LIBRARIES})
add_dependencies(test_update_parameters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


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
# install(TARGETS wabian_planner wabian_planner_node
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
