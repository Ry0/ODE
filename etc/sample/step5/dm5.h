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

typedef struct{
    dBodyID body; // ボディのID
    dGeomID geom; // ジオメトリのID
    const double *p; // x, y, z　[m]
    const double *R;   // 回転行列 要素数4x3
    double m; // 質量 [kg]
    double r,l; // 半径 [m], 長さ [m]
    const double *side; // サイズ　x,y,z
    const double *color; // 色 r,g,b
} dmObject;

void command(int cmd);               /*** キー入力の処理　***/
void start();                 /*** 前処理　***/
void setDrawStuff();          /*** 描画関数の設定 ***/
void dmAddForce(dmObject *obj, double fx, double fy, double fz);
void dmAddTorque(dmObject *obj, double fx, double fy, double fz);
void dmSetCamera(double x, double y, double z, double yaw, double pitch, double roll);
void dmClose();
void dmCreateSphere(dmObject *obj, double p[3], double R[12], double m, double r, double color[3]);
void dmCreateBox(dmObject *obj, double p[3], double R[12], double m, double side[3],  double color[3]);
void dmCreateCylinder(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3]);
void dmCreateCapsule(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3]);
void dmInit();
void dmLoop(int w, int h, void (*function)(int pause), void (*function2)(int cmd));
void dmSimStep();
void dmSimQuickStep();
void dmDraw(dmObject *obj);




