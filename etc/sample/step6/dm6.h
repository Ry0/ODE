#ifndef _DM6_H_
#define _DM6_H_

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

extern void simLoop(int cmd);
extern double R[12];

typedef struct
{
	dBodyID body;        // ボディのID
	dGeomID geom;        // ジオメトリのID
	const double *p;     // x, y, z [m]
	const double *R;     // 回転行列 要素数4x3
	double m;            // 質量 [kg]
	double r,l;          // 半径 [m], 長さ [m]
	const double *side;  // サイズ x, y, z
	const double *color; // 色 r, g, b
} dmObject;

typedef struct
{
	void (*start)();            // 初期化関数
	void (*step) (int pause);   // ステップ関数
	void (*command) (int cmd);  // キー関数
} dmFunctions;

void command(int cmd);                                  /*** キー入力の処理 ***/
void start();                                           /*** 前処理 ***/
void setDrawStuff();                                    /*** 描画関数の設定 ***/
void nearCallback(void *data, dGeomID o1, dGeomID o2);  /*** 衝突検出用コールバック関数 ***/
void dmAddForce(dmObject *obj, double fx, double fy, double fz);
void dmAddTorque(dmObject *obj, double fx, double fy, double fz);
void dmSetCamera(double x, double y, double z, double roll, double pitch, double yaw);
int dmCollision(dmObject *obj1, dmObject *obj2);
void dmClose();
void dmCreateSphere(dmObject *obj, double p[3], double R[12], double m, double r, double color[3]);
void dmCreateBox(dmObject *obj, double p[3], double R[12], double m, double side[3],  double color[3]);
void dmCreateCylinder(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3]);
void dmCreateCapsule(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3]);
void dmCreateSphere0(dmObject *obj, double p[3], double R[12], double m, double r, double color[3]);
void dmCreateBox0(dmObject *obj, double p[3], double R[12], double m, double side[3],  double color[3]);
void dmCreateCylinder0(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3]);
void dmCreateCapsule0(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3]);
void dmDestroyBox0(dmObject *obj);
void dmInit();
void dmLoop(int w, int h, dmFunctions *dmf);
void dmWorldStep();
void dmWorldQuickStep();
void dmDraw(dmObject *obj);
#endif
