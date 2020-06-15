cmake_minimum_required(VERSION 3.0.2)
project(hedefe_git)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  nav_msgs
  rospy
  std_msgs
)
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Mesafe.msg
)
## Generate services in the 'srv' folder
add_service_files(
   FILES
   VarisDurum.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)
###################################
## catkin specific configuration ##
###################################
catkin_package()
###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


