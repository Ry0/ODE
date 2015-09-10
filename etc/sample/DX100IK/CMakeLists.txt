cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -Wall
  -g
  -O2
  -std=c++11
  -DdDOUBLE
  -DdTRIMESH_ENABLED
)

project( DX100IK )

<<<<<<< HEAD
include_directories("/usr/include/X11/")
include_directories("/usr/local/include/ode/")
include_directories("/usr/local/include/drawstuff/")

find_package( OpenCV REQUIRED )

link_directories("/usr/lib/X11/")

add_executable( DX100IK DX100IK.cpp arm_struct.cpp control.cpp)
target_link_libraries( DX100IK ${OpenCV_LIBS} ode drawstuff GL GLU Xmu X11 stdc++ pthread m )
=======
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

find_package( OpenCV REQUIRED )

include_directories("/usr/local/include/ode/")
include_directories("/usr/local/include/drawstuff/")

add_executable( DX100IK DX100IK.cpp )
target_link_libraries( DX100IK ${OpenCV_LIBS} ode drawstuff ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} stdc++ pthread m )
>>>>>>> origin/For_Mac
