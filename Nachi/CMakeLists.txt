cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -Wall
  -g
  -O2
  -std=c++11
  -DdDOUBLE
  -DdTRIMESH_ENABLED
)

project( Nachi )

include_directories("/usr/include/X11/")
include_directories("/usr/local/include/ode/")
include_directories("/usr/local/include/drawstuff/")

find_package( OpenCV REQUIRED )

link_directories("/usr/lib/X11/")

add_executable( Nachi main.cpp arm_struct.cpp obstacle_struct.cpp control.cpp)
target_link_libraries( Nachi ${OpenCV_LIBS} ode drawstuff GL GLU Xmu X11 stdc++ pthread m )
