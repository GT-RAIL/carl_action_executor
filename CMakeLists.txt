cmake_minimum_required(VERSION 2.8.3)
project(carl_action_executor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  carl_dynamixel
  carl_moveit
  carl_navigation
  geometry_msgs
  message_generation
  move_base_msgs
  rail_manipulation_msgs
  std_srvs
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GeneralAction.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddAction.srv
  GetActionList.srv
  InsertAction.srv
  RemoveAction.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Execute.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(carl_action_executor src/CarlActionExecutor.cpp)
add_dependencies(carl_action_executor carl_action_executor_gencpp)
target_link_libraries(carl_action_executor ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS carl_action_executor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
