cmake_minimum_required(VERSION 2.8.3)
project(my_moves)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(actionlib REQUIRED)
#find_package(catkin REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  baxter_core_msgs
  angles
  std_msgs
  trajectory_msgs
  actionlib_msgs 
  baxter_traj_streamer 
)



catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} actionlib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs baxter_core_msgs trajectory_msgs 
  cwru_joint_space_planner baxter_kinematics cwru_srv actionlib_msgs #my_moves
  DEPENDS eigen system_lib actionlib

  #CATKIN_DEPENDS roscpp std_msgs sensor_msgs  trajectory_msgs baxter_traj_streamer    actionlib_msgs 
  #DEPENDS eigen system_lib actionlib
)


include_directories(include)

include_directories(
${Eigen_INCLUDE_DIRS}
${PROJECT_NAME}/include
${catkin_INCLUDE_DIRS}
)


add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
#cs_add_library(baxter_traj_streamer src/baxter_traj_streamer.cpp)
add_library(
    ${PROJECT_NAME}  
	src/my_moves.cpp
)   

# Executables
add_executable(do_my_moves src/do_my_moves.cpp)


#Libraries
target_link_libraries(
my_moves
baxter_traj_streamer
${catkin_LIBRARIES}
)

add_dependencies(my_moves my_moves)
add_dependencies(do_my_moves my_moves)
    