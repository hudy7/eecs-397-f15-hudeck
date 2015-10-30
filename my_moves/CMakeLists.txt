cmake_minimum_required(VERSION 2.8.3)
project(my_moves)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(actionlib REQUIRED)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  actionlib_msgs 
  baxter_traj_streamer 
)

include_directories(
${Eigen_INCLUDE_DIRS}
${PROJECT_NAME}/include

)


catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} actionlib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs  trajectory_msgs baxter_traj_streamer    actionlib_msgs 
  DEPENDS eigen system_lib actionlib
)

catkin_simple()

add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
#cs_add_library(baxter_traj_streamer src/baxter_traj_streamer.cpp)
cs_add_library(my_moves src/my_moves.cpp)   

# Executables
cs_add_executable(do_my_moves src/do_my_moves.cpp)


#Libraries
target_link_libraries(
my_moves
baxter_traj_streamer
${catkin_LIBRARIES}
)

cs_install()
cs_export()
    