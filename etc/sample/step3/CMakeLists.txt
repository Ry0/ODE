cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -Wall
  -O2
  -DdDOUBLE
  -DdTRIMESH_ENABLED
)
project( step3 )

include_directories("/usr/include/X11/")
include_directories("/usr/local/include/ode/")
include_directories("/usr/local/include/drawstuff/")

link_directories("/usr/lib/X11/")

add_executable( step3 step3.cpp dm3.cpp)
target_link_libraries( step3 ode drawstuff GL GLU Xmu X11 stdc++ pthread m)
