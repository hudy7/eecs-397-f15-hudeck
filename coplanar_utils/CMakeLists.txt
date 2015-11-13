cmake_minimum_required(VERSION 2.8.3)
project(coplanar_utils)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   
cs_add_library(coplanar_utils src/coplanar_utils.cpp)

# Executables
#cs_add_executable(coplanar_ps7 src/coplanar_ps7.cpp)
#target_link_libraries(find_coplanar_points cwru_pcl_utils ${catkin_LIBRARIES})
#target_link_libraries(coplanar_ps7 coplanar_utils ${catkin_LIBRARIES})

cs_install()
cs_export()
