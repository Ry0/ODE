cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -Wall
  -O2
  -DdDOUBLE
  -DdTRIMESH_ENABLED
)
project( step5 )
find_package( OpenCV REQUIRED )

include_directories("/usr/include/X11/")
include_directories("/usr/local/include/ode/")
include_directories("/usr/local/include/drawstuff/")

link_directories("/usr/lib/X11/")

add_executable( step5 step5.cpp dm5.cpp)
target_link_libraries( step5 ${OpenCV_LIBS} ode drawstuff GL GLU Xmu X11 stdc++ pthread m)
