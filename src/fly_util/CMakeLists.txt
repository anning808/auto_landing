cmake_minimum_required(VERSION 2.8.3)
project(fly_util)

find_package( OpenCV 4 REQUIRED )
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  mavros
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
  message_generation
  #darknet_ros_msgs
  image_transport
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
add_message_files(
  DIRECTORY msg
  FILES
  command.msg
  command_acc.msg
  ude_log.msg 
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
 

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  message_runtime
)
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###############################
##      声明可执行cpp文件     ##
###############################

###### Main File ##########


add_executable(px4_sender
  src/px4_sender.cpp
)
add_dependencies(px4_sender fly_util_gencpp)
target_link_libraries(px4_sender
  ${catkin_LIBRARIES}
)




###### Application File ##########


add_executable(autonomous_landing
  src/Application/autonomous_landing.cpp
)
add_dependencies(autonomous_landing fly_util_gencpp)
target_link_libraries(autonomous_landing
  ${catkin_LIBRARIES}
)
 

add_executable(trak src/Application/trak.cpp)
target_link_libraries(trak ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${OpenCV_LIBRARIES}) 


# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

