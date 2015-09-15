#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
//#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox dsDrawBoxD
#endif
#define NUM 7                          // リンク数

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointID      joint[NUM];              // ジョイントのID番号
dsFunctions   fn;                      // ドロースタッフの描画関数

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

MyObject rlink[NUM];                   // リンク
dReal  THETA[NUM] = {0.0};             // 関節の目標角度[rad]

/*** ロボットアームの生成 ***/
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.05, 0.55, 1.55, 2.30, 2.80, 3.15, 3.25};  // 重心 z
  dReal length[NUM] = {0.10, 1.00, 1.00, 0.50, 0.50, 0.10, 0.10};  // 長さ
  dReal weight[NUM] = {9.00, 2.00, 2.00, 1.00, 1.00, 0.50, 0.50};  // 質量
  // dReal r[NUM]      = {0.20, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06};  // 半径
  dReal x_length[NUM]      = {0.20, 0.06, 0.06, 0.06, 0.07, 0.05, 0.08};  // X軸方向の長さ
  dReal y_length[NUM]      = {0.20, 0.06, 0.06, 0.06, 0.07, 0.05, 0.08};  // y軸方向の長さ
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.00, 0.10, 1.10, 2.10, 2.60, 3.10, 3.20};  // 関節中心点 z
  dReal axis_x[NUM] = {0, 0, 0, 0, 0, 0, 0};              // 関節回転軸 x
  dReal axis_y[NUM] = {0, 0, 1, 1, 0, 1, 0};              // 関節回転軸 y
  dReal axis_z[NUM] = {1, 1, 0, 0, 1, 0, 1};              // 関節回転軸 z

  // リンクの生成
  for (int i = 0; i < NUM; i++) {
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,weight[i],3,x_length[i],y_length[i],length[i]);
    dBodySetMass(rlink[i].body, &mass);
    //rlink[i].geom = dCreateCapsule(space,r[i],length[i]);
    rlink[i].geom = dCreateBox(space,x_length[i],y_length[i],length[i]);
    dGeomSetBody(rlink[i].geom,rlink[i].body);
  }

  // ジョイントの生成とリンクへの取り付け
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], rlink[0].body, 0);
  dJointSetFixed(joint[0]);
  for (int j = 1; j < NUM; j++) {
    joint[j] = dJointCreateHinge(world, 0); // ヒンジジョイント
    dJointAttach(joint[j], rlink[j].body, rlink[j-1].body);
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j],axis_z[j]);
  }
}

/*** ロボットアームの描画 ***/
void drawArm()
{
   dReal r,length;

   for (int i = 0; i < NUM; i++ ) {       // カプセルの描画
     // dGeomCapsuleGetParams(rlink[i].geom, &r,&length);
     // dsDrawCapsule(dBodyGetPosition(rlink[i].body),
     // dBodyGetRotation(rlink[i].body),length,r);
     dGeomBoxGetLengths(rlink[i].geom, &x_length, &y_length, &length);
     dsDrawBox(dBodyGetPosition(rlink[i].body),
     dBodyGetRotation(rlink[i].body),length,r);
   }
}

/*** P制御 ***/
void Pcontrol()
{
  dReal k =  10.0, fMax = 100.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}

/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = {    3.0f, 1.3f, 0.8f};          // 視点[m]
  float hpr[3] = { -160.0f, 4.5f, 0.0f};          // 視線[°]
  dsSetViewpoint(xyz, hpr);                       // 視点と視線の設定
}

/*** キー入力関数 ***/
void command(int cmd)
{
  switch (cmd) {
    case 'j': THETA[1] += M_PI/180; break;     // jキー
    case 'f': THETA[1] -= M_PI/180; break;     // fキー
    case 'k': THETA[2] += M_PI/180; break;     // kキー
    case 'd': THETA[2] -= M_PI/180; break;     // dキー
    case 'l': THETA[3] += M_PI/180; break;     // lキー
    case 's': THETA[3] -= M_PI/180; break;     // sキー
    case 'u': THETA[4] += M_PI/180; break;     // uキー
    case 'r': THETA[4] -= M_PI/180; break;     // rキー
    case 'i': THETA[5] += M_PI/180; break;     // iキー
    case 'e': THETA[5] -= M_PI/180; break;     // eキー
    case 'o': THETA[6] += M_PI/180; break;     // oキー
    case 'w': THETA[6] -= M_PI/180; break;     // wキー
  }
}

/*** シミュレーションループ ***/
void simLoop(int pause)
{
  Pcontrol();                                  // P制御
  dWorldStep(world, 0.01);                     // 動力学計算
  drawArm();                                   // ロボットの描画
}

/*** ドロースタッフの設定 ***/
void setDrawStuff()
{
  fn.version = DS_VERSION;                     // バージョン番号
  fn.start   = &start;                         // start関数
  fn.step    = &simLoop;                       // simLoop関数
  fn.command = &command;                       // command関数
  fn.path_to_textures = "./textures";
}

int main(int argc, char **argv)
{
  dInitODE();                                     // ODEの初期化
  setDrawStuff();
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  ground       = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定
  makeArm();                                      // アームの生成
  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ
  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE();                                    // ODEの終了
  return 0;
}