cmake_minimum_required(VERSION 2.8.3)
project(oni_vicon_player)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    std_msgs
    message_generation
    actionlib_msgs
    actionlib
    depth_sensor_vicon_calibration
    image_transport
    oni_vicon_common
)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
  DIRECTORY srv
  FILES
  Pause.srv
  SeekFrame.srv
  SetPlaybackSpeed.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  Open.action
  Play.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES oni_vicon_player
    CATKIN_DEPENDS actionlib_msgs actionlib depth_sensor_vicon_calibration image_transport oni_vicon_common
    DEPENDS OpenNI
)

###########
## Build ##
###########
set(oni_vicon_player_SOURCES
    src/${PROJECT_NAME}/oni_player.cpp
    src/${PROJECT_NAME}/vicon_player.cpp
    src/${PROJECT_NAME}/oni_vicon_player.cpp
    src/${PROJECT_NAME}/oni_vicon_playback_server.cpp
)

set(oni_vicon_player_HEADERS
    include/${PROJECT_NAME}/oni_player.hpp
    include/${PROJECT_NAME}/vicon_player.hpp
    include/${PROJECT_NAME}/oni_vicon_player.hpp
    include/${PROJECT_NAME}/oni_vicon_playback_server.hpp
    include/${PROJECT_NAME}/exceptions.hpp
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    "/usr/include/ni/" # FIXME superbly uggly
)

add_library(oni_vicon_player ${oni_vicon_player_SOURCES} ${oni_vicon_player_HEADERS})
target_link_libraries(oni_vicon_player
    ${catkin_LIBRARIES}
    OpenNI
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

add_executable(oni_vicon_player_node src/${PROJECT_NAME}/oni_vicon_player_node.cpp)
target_link_libraries(oni_vicon_player_node
    oni_vicon_player
    ${catkin_LIBRARIES}
    OpenNI
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

## Mark executables and/or libraries for installation
# install(TARGETS oni_vicon_player oni_vicon_player_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_oni_vicon_player.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
