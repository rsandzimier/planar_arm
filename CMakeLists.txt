# CMakeLists.txt
# Author: Ryan Sandzimier

cmake_minimum_required (VERSION 3.16.3)
project (planar_arm)

include_directories(include)

add_library(planar_arm src/environment.cpp src/kinematics.cpp src/collision.cpp src/planner.cpp src/utils.cpp)

add_executable (demo src/demo.cpp)

target_link_libraries(demo planar_arm sfml-graphics sfml-window sfml-system)