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

using namespace std;

#define IK
// #define Path

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawLine     dsDrawLineD
#endif

#define NUM 10                          // リンク数

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

typedef struct {
  double x;
  double y;
  double z;
} POINT;

#endif