#ifndef ARM_STRUCT_H_
#define ARM_STRUCT_H_
#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <iostream>
#include <fstream>
#include <vector>

#include "draw_arms.h"

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

#endif
