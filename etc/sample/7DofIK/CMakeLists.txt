cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -Wall
  -g
  -O2
  -std=c++11
  -DdDOUBLE
  -DdTRIMESH_ENABLED
)

project( 7DofIK )

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

find_package( OpenCV REQUIRED )

include_directories("/usr/local/include/ode/")
include_directories("/usr/local/include/drawstuff/")

add_executable( 7DofIK 7DofIK.cpp )
target_link_libraries( 7DofIK ${OpenCV_LIBS} ode drawstuff ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} stdc++ pthread m )