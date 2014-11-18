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

link_directories("/usr/lib/X11/")

add_executable( Nachi main.cpp draw_arms.cpp)
target_link_libraries( Nachi ode drawstuff GL GLU Xmu X11 stdc++ pthread m)
