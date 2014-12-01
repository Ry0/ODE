#ifndef AREA_STRUCT_H_
#define AREA_STRUCT_H_

#include <iostream>
#include <fstream>
#include <ostream>

#include <iterator>
#include <vector>
#include <string>
#include <algorithm>

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <stdio.h>
#include <stdlib.h>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

struct PlotData {
  double t;
  double x;
  double y;
  double z;
};

typedef struct {
  double x;
  double y;
  double z;
} POINT;

#endif
