#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawSphere   dsDrawSphereD
#define dsDrawBox      dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif


extern double R[12];
//extern dMatrix3 R;

void start();                 /*** 前処理　***/

/*** 描画関数の設定 ***/
void setDrawStuff(void (*function)(int pause), void (*function2)(int cmd));
void dmLoop(int w, int h, void (*function)(int pause), void (*function2)(int cmd));

