#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>                     // ODE
#include <drawstuff/drawstuff.h> // ODEの描画ライブラリ

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#endif
#define NUM 4         // リンク数(土台含む)

dWorldID      world;       // 動力学計算世界
dSpaceID      space;
dGeomID       ground;
dJointGroupID contactgroup;
dJointID      joint[NUM];  // 関節    joint[0]は土台と地面を固定
dsFunctions fn;

typedef struct{
  dBodyID body;
  dGeomID geom;
}MyObject;

MyObject rlink[NUM];
dReal THETA[NUM] = {0.0};


void makeArm(){
  dMass mass;
  dReal x[NUM] = {0.00, 0.00, 0.00, 0.00};
  dReal y[NUM] = {0.00, 0.00, 0.00, 0.00};
  dReal z[NUM] = {0.05, 0.50, 1.50, 2.50};
  dReal length[NUM] = {0.10, 0.90, 1.00, 1.00};
  dReal weight[NUM] = {9.00, 2.00, 2.00, 2.00};
  dReal r[NUM] = {0.20, 0.04, 0.04, 0.04};
  dReal c_x[NUM] = {0.00, 0.00, 0.00, 0.00};
  dReal c_y[NUM] = {0.00, 0.00, 0.00, 0.00};
  dReal c_z[NUM] = {0.00, 0.10, 1.00, 2.00};
  dReal axis_x[NUM]  = {0, 0, 0, 0};  //回転軸
  dReal axis_y[NUM]  = {0, 0, 1, 1};
  dReal axis_z[NUM]  = {1, 1, 0, 0};



  for(int i=0;i<NUM;i++){
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass, weight[i], 3, r[i], length[i]);
    dBodySetMass(rlink[i].body, &mass);
    rlink[i].geom = dCreateCapsule(space, r[i], length[i]);
    dGeomSetBody(rlink[i].geom, rlink[i].body);
  }


  joint[0] = dJointCreateFixed(world, 0); // 固定関節（土台と地面の固定）
  dJointAttach(joint[0], rlink[0].body, 0);     // 固定関節の取付け
  dJointSetFixed(joint[0]);               // 固定関節の設定
  for(int j = 1; j <NUM; j++) {
    joint[j] = dJointCreateHinge(world, 0); // ヒンジ関節生成
    dJointAttach(joint[j], rlink[j].body, rlink[j-1].body); // 関節の取付け
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]); //関節中心の設定
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]); // 関節回転軸の設定
  }
}


void drawArm()
{
  dReal r,length;

  for(int i=0;i<NUM;i++){
    dGeomCapsuleGetParams(rlink[i].geom, &r, &length);
    dsDrawCapsule(dBodyGetPosition(rlink[i].body), dBodyGetRotation(rlink[i].body), length, r);
  }
}



void Pcontrol()
{  /***  P制御  ****/
   dReal k =  10.0,  fMax  = 100.0; // k1:比例ゲイン,  fMax：最大トルク[Nm]

   for (int j = 1; j <NUM; j++) {
     dReal tmp = dJointGetHingeAngle(joint[j]);  // 現在の関節角[rad]
     double z = THETA[j] - tmp;  // z: 残差=目標関節角－現在関節角
     dJointSetHingeParam(joint[j],  dParamVel,  k*z); // 角速度の設定
     dJointSetHingeParam(joint[j], dParamFMax, fMax); // 最大トルクの設定
   }
}


void start()
{ /*** 描画APIの初期化 ***/
  float xyz[3] = {3.0, 1.3, 0.8};   // 視点x, y, z　[m]
  float hpr[3] = { -160.0, 4.50, 0.00};  // 視線(heading, pitch, roll)　[°]
  dsSetViewpoint(xyz, hpr);               // 視点と視線の設定
}


void command(int cmd)
{ /***  キー操作関数 ***/
  switch (cmd) {
  case 'j': THETA[1] += M_PI/180; break;
  case 'f': THETA[1] -= M_PI/180; break;
  case 'k': THETA[2] += M_PI/180; break;
  case 'd': THETA[2] -= M_PI/180; break;
  case 'l': THETA[3] += M_PI/180; break;
  case 's': THETA[3] -= M_PI/180; break;
  }
}


// シミュレーションループ。簡単にするため衝突検出に関するコードは省略。
void simLoop(int pause){
  Pcontrol();
  dWorldStep(world, 0.01);
  drawArm();
}


void setDrawStuff()
{
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.path_to_textures = "./textures";
}

int main(int argc, char *argv[])
{
  setDrawStuff();
  world = dWorldCreate();  // 動力学計算世界の生成
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground = dCreatePlane(space, 0, 0, 1, 0);
  dWorldSetGravity(world, 0, 0, -9.8);     // 重力設定
  makeArm();
  dsSimulationLoop(argc, argv, 640, 480, &fn); // シミュレーションループ
  dSpaceDestroy(space);
  dWorldDestroy(world);
  return 0;
}
