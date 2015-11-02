cmake_minimum_required(VERSION 2.8.3)
project(do_my_moves)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

cs_add_executable(do_my_moves src/do_my_moves.cpp)

target_link_libraries(do_my_moves my_moves ${catkin_LIBRARIES})

cs_install()
cs_export()
    