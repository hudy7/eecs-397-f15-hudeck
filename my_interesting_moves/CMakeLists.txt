cmake_minimum_required(VERSION 2.8.3)
project(my_interesting_moves)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(actionlib REQUIRED)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  baxter_kinematics
  cwru_joint_space_planner
  cwru_srv
  actionlib_msgs
  baxter_traj_streamer  
)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${PROJECT_NAME}/include
  #${baxter_kinematics_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
add_action_files(
  #DIRECTORY baxter_traj_streamer
  FILES
  traj.action
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs trajectory_msgs std_msgs cwru_srv)
#   sensor_msgs
#   std_msgs
#   trajectory_msgs
# )

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} cwru_joint_space_planner actionlib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs baxter_core_msgs trajectory_msgs cwru_joint_space_planner baxter_kinematics cwru_srv   actionlib_msgs 
  DEPENDS eigen system_lib actionlib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library

 add_library(
  ${PROJECT_NAME}  
 
  src/baxter_flex.cpp

 )

## Declare a cpp executable

#add_executable(traj_interpolator_as src/traj_interpolator_as.cpp)

add_executable(baxter_flex src/baxter_flex.cpp)
#add_executable(traj_interpolator_as src/traj_interpolator_as.cpp)
#add_executable(baxter_traj_streamer_node src/baxter_traj_streamer_node.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(traj_action_client_pre_pose baxter_traj_streamer_generate_messages_cpp)
add_dependencies(traj_interpolator_as baxter_traj_streamer_generate_messages_cpp)
add_dependencies(my_interesting_moves baxter_core_msgs_gencpp)
add_dependencies(baxter_traj_streamer baxter_traj_streamer_cpp)

#target_link_libraries(traj_interpolator_as baxter_traj_streamer  ${catkin_LIBRARIES})
#target_link_libraries(traj_action_client_pre_pose baxter_traj_streamer ${catkin_LIBRARIES})


    
