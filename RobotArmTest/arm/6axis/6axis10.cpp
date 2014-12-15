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
dJointID      joint[7];              // ジョイントのID番号
dJointID      bodyjoint1[2], bodyjoint2[2], bodyjoint4[2], bodyjoint6[3];
dsFunctions   fn;                      // ドロースタッフの描画関数

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

MyObject base, bodyparts1[3], bodyparts2[3], bodyparts3, bodyparts4[3], bodyparts5, bodyparts6[4];     // リンク
dReal  THETA[7] = {0.0};             // 関節の目標角度[rad]

/*** ロボットアームの生成 ***/
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ

  dReal  parts1_mass[3]  = {0.1, 0.2, 0.1};// 質量
  dReal  parts1_x_length[3] = {0.067, 0.1, 0.067};
  dReal  parts1_y_length[3] = {0.15, 0.15, 0.15};
  dReal  parts1_z_length[3] = {0.195, 0.07, 0.195};
  dReal  parts1_start_x[3] = {-0.08035, 0, 0.08035};// 重心 x
  dReal  parts1_start_y[3] = {0, 0, 0};// 重心 y
  dReal  parts1_start_z[3] = {0.2975, 0.235, 0.2975};// 重心 z
  dReal  parts1_hinge_x[2] = {-0.05, 0.05};

  dReal  parts2_mass[3]  = {0.2, 0.4, 0.2};// 質量
  dReal  parts2_x_length[3] = {0.06, 0.1, 0.06};
  dReal  parts2_y_length[3] = {0.1, 0.1, 0.1};
  dReal  parts2_z_length[3] = {0.27, 0.26, 0.27};
  dReal  parts2_start_x[3] = {-0.08, 0, 0.08};// 重心 x
  dReal  parts2_start_y[3] = {0, 0, 0};// 重心 y
  dReal  parts2_start_z[3] = {0.59, 0.425, 0.59};// 重心 z
  dReal  parts2_hinge_x[2] = {-0.05, 0.05};

  dReal  parts3_mass  = 0.2;// 質量
  dReal  parts3_x_length = 0.1;
  dReal  parts3_y_length = 0.1;
  dReal  parts3_z_length = 0.2;
  dReal  parts3_start_x = 0;// 重心 x
  dReal  parts3_start_y = 0;// 重心 y
  dReal  parts3_start_z = 0.725;// 重心 z

  dReal  parts4_mass[3]  = {0.2, 0.4, 0.2};// 質量
  dReal  parts4_x_length[3] = {0.02, 0.08, 0.02};
  dReal  parts4_y_length[3] = {0.12, 0.12, 0.12};
  dReal  parts4_z_length[3] = {0.23, 0.14, 0.23};
  dReal  parts4_start_x[3] = {-0.05, 0, 0.05};// 重心 x
  dReal  parts4_start_y[3] = {0, 0, 0};// 重心 y
  dReal  parts4_start_z[3] = {0.94, 0.895, 0.94};// 重心 z
  dReal  parts4_hinge_x[2] = {-0.04, 0.04};

  dReal  parts5_mass  = 0.1;// 質量
  dReal  parts5_x_length = 0.08;
  dReal  parts5_y_length = 0.08;
  dReal  parts5_z_length = 0.113;
  dReal  parts5_start_x = 0;// 重心 x
  dReal  parts5_start_y = 0;// 重心 y
  dReal  parts5_start_z = 1.0315;// 重心 z

  dReal  parts6_mass[3]  = {0.2, 0.05, 0.2};// 質量
  dReal  parts6_start_x[3] = {0, 0, 0};// 重心 x
  dReal  parts6_start_y[3] = {0, 0, 0};// 重心 y
  dReal  parts6_start_z[3] = {1.123, 1.198, 1.253};// 重心 z
  dReal  parts6_z_length[3] = {0.07, 0.08, 0.03};  // 長さ
  dReal  parts6_r[3]      = {0.04, 0.02, 0.05};  // 半径
  dReal  parts6_hinge_z[2] = {1.158, 1.238};

  //ベースの生成
  base.body = dBodyCreate(world);
  dBodySetPosition(base.body, 0, 0, 0.1);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass, 0.6, 3, 0.12, 0.2);
  dBodySetMass(base.body, &mass);
  base.geom = dCreateCylinder(space, 0.12, 0.2);
  dGeomSetBody(base.geom,base.body);


//パーツ1の作成
  for (int i = 0; i < 3; i++) {
    bodyparts1[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,parts1_mass[i], parts1_x_length[i], parts1_y_length[i], parts1_z_length[i]);
    dBodySetMass(bodyparts1[i].body, &mass);

    bodyparts1[i].geom = dCreateBox(space, parts1_x_length[i], parts1_y_length[i], parts1_z_length[i]);
    dGeomSetBody(bodyparts1[i].geom, bodyparts1[i].body);
    dBodySetPosition(bodyparts1[i].body, parts1_start_x[i], parts1_start_y[i], parts1_start_z[i]);
}

//パーツ2の作成
  for (int i = 0; i < 3; i++) {
    bodyparts2[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,parts2_mass[i], parts2_x_length[i], parts2_y_length[i], parts2_z_length[i]);
    dBodySetMass(bodyparts1[i].body, &mass);

    bodyparts2[i].geom = dCreateBox(space, parts2_x_length[i], parts2_y_length[i], parts2_z_length[i]);
    dGeomSetBody(bodyparts2[i].geom, bodyparts2[i].body);
    dBodySetPosition(bodyparts2[i].body, parts2_start_x[i], parts2_start_y[i], parts2_start_z[i]);
}

//パーツ3の作成
  bodyparts3.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,parts3_mass, parts3_x_length, parts3_y_length, parts3_z_length);
  dBodySetMass(bodyparts3.body, &mass);

  bodyparts3.geom = dCreateBox(space, parts3_x_length, parts3_y_length, parts3_z_length);
  dGeomSetBody(bodyparts3.geom, bodyparts3.body);
  dBodySetPosition(bodyparts3.body, parts3_start_x, parts3_start_y, parts3_start_z);

//パーツ4の作成
  for (int i = 0; i < 3; i++) {
    bodyparts4[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,parts4_mass[i], parts4_x_length[i], parts4_y_length[i], parts4_z_length[i]);
    dBodySetMass(bodyparts4[i].body, &mass);

    bodyparts4[i].geom = dCreateBox(space, parts4_x_length[i], parts4_y_length[i], parts4_z_length[i]);
    dGeomSetBody(bodyparts4[i].geom, bodyparts4[i].body);
    dBodySetPosition(bodyparts4[i].body, parts4_start_x[i], parts4_start_y[i], parts4_start_z[i]);
}

//パーツ5の作成
  bodyparts5.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,parts5_mass, parts5_x_length, parts5_y_length, parts5_z_length);
  dBodySetMass(bodyparts5.body, &mass);

  bodyparts5.geom = dCreateBox(space, parts5_x_length, parts5_y_length, parts5_z_length);
  dGeomSetBody(bodyparts5.geom, bodyparts5.body);
  dBodySetPosition(bodyparts5.body, parts5_start_x, parts5_start_y, parts5_start_z);

//パーツ6の作成
  for (int i = 0; i < 3; i++) {
    bodyparts6[i].body = dBodyCreate(world);
    dBodySetPosition(bodyparts6[i].body, parts6_start_x[i], parts6_start_y[i], parts6_start_z[i]);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass, parts6_mass[i], 3, parts6_r[i], parts6_z_length[i]);
    dBodySetMass(bodyparts6[i].body, &mass);
    bodyparts6[i].geom = dCreateCapsule(space, parts6_r[i], parts6_z_length[i]);
    dGeomSetBody(bodyparts6[i].geom, bodyparts6[i].body);
}
    bodyparts6[3].body = dBodyCreate(world);
    dBodySetPosition(bodyparts6[3].body, 0, 0, 1.268);
    dMassSetZero(&mass);
    dMassSetSphereTotal(&mass, 0.3, 0.04);
    dBodySetMass(bodyparts6[3].body, &mass);
    bodyparts6[3].geom = dCreateSphere(space, 0.04);
    dGeomSetBody(bodyparts6[3].geom, bodyparts6[3].body);


//パーツ1の合体
  for (int i = 0; i < 2; i++) {
    bodyjoint1[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint1[i], bodyparts1[i].body, bodyparts1[i+1].body);
    dJointSetHingeAxis(bodyjoint1[0],1, 0, 0);
    dJointSetHingeAnchor(bodyjoint1[i], parts1_hinge_x[i], 0, 0.235);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint1[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint1[i],dParamHiStop, 0);
  }

//パーツ2の合体
  for (int i = 0; i < 2; i++) {
    bodyjoint2[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint2[i], bodyparts2[i].body, bodyparts2[i+1].body);
    dJointSetHingeAxis(bodyjoint2[0],1, 0, 0);
    dJointSetHingeAnchor(bodyjoint2[i], parts2_hinge_x[i], 0, 0.505);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint2[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint2[i],dParamHiStop, 0);
  }

//パーツ4の合体
  for (int i = 0; i < 2; i++) {
    bodyjoint4[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint4[i], bodyparts4[i].body, bodyparts4[i+1].body);
    dJointSetHingeAxis(bodyjoint4[0],1, 0, 0);
    dJointSetHingeAnchor(bodyjoint4[i], parts4_hinge_x[i], 0, 0.895);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint4[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint4[i],dParamHiStop, 0);
  }

//パーツ6の合体
  for (int i = 0; i < 2; i++) {
    bodyjoint6[i] = dJointCreateHinge(world, 0); // ヒンジジョイント
    dJointAttach(bodyjoint6[i], bodyparts6[i].body, bodyparts6[i+1].body);
    dJointSetHingeAnchor(bodyjoint6[i], 0, 0, parts6_hinge_z[i]);
    dJointSetHingeAxis(bodyjoint6[i], 0, 0, 1);
    dJointSetHingeParam(bodyjoint6[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint6[i],dParamHiStop, 0);
}
    bodyjoint6[2] = dJointCreateHinge(world, 0); // ヒンジジョイント
    dJointAttach(bodyjoint6[2], bodyparts6[2].body, bodyparts6[3].body);
    dJointSetHingeAnchor(bodyjoint6[2], 0, 0, 1.268);
    dJointSetHingeAxis(bodyjoint6[2], 0, 0, 1);



  // ジョイントの生成とリンクへの取り付け
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], base.body, 0);


  joint[1] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[1], base.body, bodyparts1[1].body);
  dJointSetHingeAnchor(joint[1], 0, 0, 0.2);
  dJointSetHingeAxis(joint[1], 0, 0, 1);

  joint[2] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[2], bodyparts1[0].body, bodyparts2[1].body);
  dJointSetHingeAnchor(joint[2], -0.05, 0, 0.345);
  dJointSetHingeAxis(joint[2], 1, 0, 0);

  joint[3] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[3], bodyparts2[0].body, bodyparts3.body);
  dJointSetHingeAnchor(joint[3], -0.05, 0, 0.675);
  dJointSetHingeAxis(joint[3], 1, 0, 0);

  joint[4] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[4], bodyparts3.body, bodyparts4[1].body);
  dJointSetHingeAnchor(joint[4], 0, 0, 0.825);
  dJointSetHingeAxis(joint[4], 0, 0, 1);

  joint[5] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[5], bodyparts4[0].body, bodyparts5.body);
  dJointSetHingeAnchor(joint[5], 0, 0, 1.015);
  dJointSetHingeAxis(joint[5], 1, 0, 0);

  joint[6] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[6], bodyparts5.body, bodyparts6[0].body);
  dJointSetHingeAnchor(joint[6], 0, 0, 1.088);
  dJointSetHingeAxis(joint[6], 0, 0, 1);
}

/*** ロボットアームの描画 ***/
void drawArm()
{
  // dReal r,length;
  dReal parts1_length_02[] = {0.067, 0.15, 0.195};
  dReal parts1_length_1[] = {0.1, 0.15, 0.07};

  dReal parts2_length_02[] = {0.06, 0.1, 0.27};
  dReal parts2_length_1[] = {0.1, 0.1, 0.26};

  dReal parts3_length[] = {0.1, 0.1, 0.2};

  dReal parts4_length_02[] = {0.02, 0.12, 0.23};
  dReal parts4_length_1[] = {0.08, 0.12, 0.14};

  dReal parts5_length[] = {0.08, 0.08, 0.113};

  dReal parts6_length[3] = {0.07, 0.08, 0.03};
  dReal parts6_r[3] = {0.04, 0.02, 0.05};

  dsDrawCylinder(dBodyGetPosition(base.body), dBodyGetRotation(base.body),0.2, 0.12);

  dsDrawBox(dBodyGetPosition(bodyparts1[0].body), dBodyGetRotation(bodyparts1[0].body), parts1_length_02);
  dsDrawBox(dBodyGetPosition(bodyparts1[1].body), dBodyGetRotation(bodyparts1[1].body), parts1_length_1);
  dsDrawBox(dBodyGetPosition(bodyparts1[2].body), dBodyGetRotation(bodyparts1[2].body), parts1_length_02);

  dsDrawBox(dBodyGetPosition(bodyparts2[0].body), dBodyGetRotation(bodyparts2[0].body), parts2_length_02);
  dsDrawBox(dBodyGetPosition(bodyparts2[1].body), dBodyGetRotation(bodyparts2[1].body), parts2_length_1);
  dsDrawBox(dBodyGetPosition(bodyparts2[2].body), dBodyGetRotation(bodyparts2[2].body), parts2_length_02);

  dsDrawBox(dBodyGetPosition(bodyparts3.body), dBodyGetRotation(bodyparts3.body), parts3_length);

  dsDrawBox(dBodyGetPosition(bodyparts4[0].body), dBodyGetRotation(bodyparts4[0].body), parts4_length_02);
  dsDrawBox(dBodyGetPosition(bodyparts4[1].body), dBodyGetRotation(bodyparts4[1].body), parts4_length_1);
  dsDrawBox(dBodyGetPosition(bodyparts4[2].body), dBodyGetRotation(bodyparts4[2].body), parts4_length_02);

  dsDrawBox(dBodyGetPosition(bodyparts5.body), dBodyGetRotation(bodyparts5.body), parts5_length);

  dsDrawCylinder(dBodyGetPosition(bodyparts6[0].body),dBodyGetRotation(bodyparts6[0].body), parts6_length[0], parts6_r[0]);
  dsDrawCylinder(dBodyGetPosition(bodyparts6[1].body),dBodyGetRotation(bodyparts6[1].body), parts6_length[1], parts6_r[1]);
  dsDrawCylinder(dBodyGetPosition(bodyparts6[2].body),dBodyGetRotation(bodyparts6[2].body), parts6_length[2], parts6_r[2]);
  dsDrawSphere(dBodyGetPosition(bodyparts6[3].body),dBodyGetRotation(bodyparts6[3].body), 0.04);

}

/*** P制御 ***/
void Pcontrol()
{
  dReal k =  10.0, fMax = 100.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < 7; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}

/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = {    1.5f, 0.65f, 0.4f};          // 視点[m]
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
