#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif
//#define NUM 4                          // リンク数

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointID      joint[3];              // ジョイントのID番号
dJointID      bodyjoint1[2];
dsFunctions   fn;                      // ドロースタッフの描画関数

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

MyObject base, bodyparts1[3], bodyparts2;                   // リンク
dReal  THETA[3] = {0.0};             // 関節の目標角度[rad]

/*** ロボットアームの生成 ***/
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ
  // dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 x
  // dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 y
  // dReal z[NUM]      = {0.05, 0.50, 1.50, 2.50};  // 重心 z
  // dReal length[NUM] = {0.10, 0.90, 1.00, 1.00};  // 長さ
  // dReal weight[NUM] = {9.00, 2.00, 2.00, 2.00};  // 質量
  dReal  part1_mass[3]  = {0.1, 0.2, 0.1};// 質量
  dReal  part1_x_length[3] = {0.1, 0.2, 0.1};
  dReal  part1_y_length[3] = {0.2, 0.2, 0.2};
  dReal  part1_z_length[3] = {0.4, 0.2, 0.4};
  dReal  part1_start_x[3] = {-0.15, 0, 0.15};
  dReal  part1_start_y[3] = {0, 0, 0};
  dReal  part1_start_z[3] = {0.6, 0.5, 0.6};

  dReal  part2_mass  = 0.2;// 質量
  dReal  part2_x_length = 0.2;
  dReal  part2_y_length = 0.2;
  dReal  part2_z_length = 0.6;
  dReal  part2_start_x = 0;
  dReal  part2_start_y = 0;
  dReal  part2_start_z = 1.05;
  // dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  // dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  // dReal c_z[NUM]    = {0.00, 0.10, 1.00, 2.00};  // 関節中心点 z
  // dReal axis_x[NUM] = {0, 0, 0, 0};              // 関節回転軸 x
  // dReal axis_y[NUM] = {0, 0, 1, 1};              // 関節回転軸 y
  // dReal axis_z[NUM] = {1, 1, 0, 0};              // 関節回転軸 z

//   for (int i = 0; i < 3; i++) {
//     bodyparts1[i].body  = dBodyCreate(world);
//     dMassSetZero(&mass);
//     dMassSetBoxTotal(&mass,part1_mass[i], part1_x_length[i], part1_y_length[i], part1_z_length[i]);
//     dBodySetMass(bodyparts1[i].body, &mass);

//     bodyparts1[i].geom = dCreateBox(space, part1_x_length[i], part1_y_length[i], part1_z_length[i]);
//     dGeomSetBody(bodyparts1[i].geom, bodyparts1[i].body);
//     dBodySetPosition(bodyparts1[i].body, part1_start_x[i], part1_start_y[i], part1_start_z[i]);
// }

//   bodyparts2.body  = dBodyCreate(world);
//   dMassSetZero(&mass);
//   dMassSetBoxTotal(&mass,part2_mass, part2_x_length, part2_y_length, part2_z_length);
//   dBodySetMass(bodyparts2.body, &mass);

//   bodyparts2.geom = dCreateBox(space, part2_x_length, part2_y_length, part2_z_length);
//   dGeomSetBody(bodyparts2.geom, bodyparts2.body);
//   dBodySetPosition(bodyparts2.body, part2_start_x, part2_start_y, part2_start_z);

//   dReal hinge_x[2] = {-0.1, 0.1};
//   for (int i = 0; i < 2; i++) {
//     bodyjoint1[i] = dJointCreateHinge(world,0);
//     dJointAttach(bodyjoint1[i], bodyparts1[i].body, bodyparts1[i+1].body);
//     dJointSetHingeAxis(bodyjoint1[0],1, 0, 0);
//     dJointSetHingeAnchor(bodyjoint1[i], hinge_x[i], 0, 0.1);
//     dJointSetHingeParam(bodyjoint1[i],dParamLoStop, 0);
//     dJointSetHingeParam(bodyjoint1[i],dParamHiStop, 0);
//   }

  // リンクの生成
  base.body = dBodyCreate(world);
  dBodySetPosition(base.body, 0.0, 0.0, 0.10);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass,9.0, 3, 0.12, 0.2);
  dBodySetMass(base.body, &mass);
  base.geom = dCreateCapsule(space, 0.12, 0.2);
  dGeomSetBody(base.geom, base.body);

  // ジョイントの生成とリンクへの取り付け
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], base.body, 0);
  dJointSetFixed(joint[0]);

}

    // joint[1] = dJointCreateHinge(world, 0); // ヒンジジョイント
    // dJointAttach(joint[1], base.body, bodyparts1[1].body);
    // dJointSetHingeAnchor(joint[1], 0, 0, 0.4);
    // dJointSetHingeAxis(joint[1], 0, 0, 1);

    // joint[2] = dJointCreateHinge(world, 0); // ヒンジジョイント
    // dJointAttach(joint[2], bodyparts1[0].body, bodyparts2.body);
    // dJointSetHingeAnchor(joint[2], 0.1, 0, 0.75);
    // dJointSetHingeAxis(joint[2], 1, 0, 0);

/*** ロボットアームの描画 ***/
void drawArm()
{
  dReal r,length;
  dReal sides02[] = {0.1,0.2,0.4};
  dReal sides1[] = {0.2,0.2,0.2};
  dReal sides3[] = {0.2,0.2,0.6};

/*** ロボットアームの描画 ***/
  dGeomCapsuleGetParams(base.geom, &r, &length);
  //dsDrawCylinder(dBodyGetPosition(base.body), dBodyGetRotation(base.body),0.4,0.2);
  dsDrawCylinder(dBodyGetPosition(base.body), dBodyGetRotation(base.body), length, r);
  // dsDrawBox(dBodyGetPosition(bodyparts1[0].body),dBodyGetRotation(bodyparts1[0].body),sides02);
  // dsDrawBox(dBodyGetPosition(bodyparts1[1].body),dBodyGetRotation(bodyparts1[1].body),sides1);
  // dsDrawBox(dBodyGetPosition(bodyparts1[2].body),dBodyGetRotation(bodyparts1[2].body),sides02);

  // dsDrawBox(dBodyGetPosition(bodyparts2.body),dBodyGetRotation(bodyparts2.body),sides3);
}

/*** P制御 ***/
void Pcontrol()
{
  dReal k =  10.0, fMax = 100.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < 3; j++) {
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
    // case 'l': THETA[3] += M_PI/180; break;     // lキー
    // case 's': THETA[3] -= M_PI/180; break;     // sキー
  }
}

/*** シミュレーションループ ***/
void simLoop(int pause)
{
  //Pcontrol();                                  // P制御
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
  fn.path_to_textures = "textures";
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
