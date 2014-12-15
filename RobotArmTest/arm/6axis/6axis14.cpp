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
dJointID      bodyjoint1[2], bodyjoint2[2], bodyjoint4[2], bodyjoint6[2];
dBodyID       sensor;                  // センサ用のボディID
dJointID      sensor_joint;            // センサ固定用の関節

dsFunctions   fn;                      // ドロースタッフの描画関数

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

MyObject base, bodyparts1[3], bodyparts2[3], bodyparts3, bodyparts4[3], bodyparts5, bodyparts6[3];     // リンク
dReal  THETA[7] = {0.0};             // 関節の目標角度[rad]

int  ANSWER = 1;              // 逆運動学の解

dReal P[3] = {0.2, 0.2, 0.3};             // 先端の位置

// 有顔ベクトル(a,b)
dReal a[3];//?わっからーん
dReal b[3] = {0.0, 0.0, 1.0};//?わっからーん
dReal T[2] = {M_PI, 0.0};
dReal l[7] = {0.20, 0.145, 0.33, 0.34, 0.34, 0.073, 0.18};   // リンクの長さ[m]

// センサの作成
void makeSensor()
{
  dMass mass;
  double sx = 0.0, sy = 0.0, sz = 1.268;  // センサの初期座標[m]
  double r = 0.04, weight = 0.01; // センサのサイズ[m]と重量[kg]

  sensor = dBodyCreate(world);          // センサの生成
  dBodySetPosition(sensor,sx,sy,sz);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass, weight, r);
  dBodySetMass(sensor, &mass);

  sensor_joint = dJointCreateFixed(world, 0); // 固定ジョイントの生成
  dJointAttach(sensor_joint, bodyparts6[2].body, sensor); // 先端リンクと結合
  dJointSetFixed(sensor_joint);
}

// センサ位置の表示
void printSensorPosition()
{
  double *pos = (double *) dBodyGetPosition(sensor);
  printf("Current Position: x=%6.2f y=%6.2f z=%6.2f \n",pos[0],pos[1],pos[2]);
  //printf("%6.2f\t%6.2f\t%6.2f\t",pos[0],pos[1],pos[2]);
  // printf("P : x=%5.2f y=%5.2f z=%5.2f \n",P[0],P[1],P[2]);
}

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
  dBodySetPosition(base.body, 0.0, 0.0, 0.10);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass,9.0, 3, 0.12, 0.2);
  dBodySetMass(base.body, &mass);
  base.geom = dCreateCapsule(space, 0.12, 0.2);
  dGeomSetBody(base.geom, base.body);


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
    // bodyparts6[3].body = dBodyCreate(world);
    // dBodySetPosition(bodyparts6[3].body, 0, 0, 1.268);
    // dMassSetZero(&mass);
    // dMassSetSphereTotal(&mass, 0.3, 0.04);
    // dBodySetMass(bodyparts6[3].body, &mass);
    // bodyparts6[3].geom = dCreateSphere(space, 0.04);
    // dGeomSetBody(bodyparts6[3].geom, bodyparts6[3].body);


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
    // bodyjoint6[2] = dJointCreateHinge(world, 0); // ヒンジジョイント
    // dJointAttach(bodyjoint6[2], bodyparts6[2].body, bodyparts6[3].body);
    // dJointSetHingeAnchor(bodyjoint6[2], 0, 0, 1.268);
    // dJointSetHingeAxis(bodyjoint6[2], 0, 0, 1);



  // ジョイントの生成とリンクへの取り付け
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], base.body, 0);
  dJointSetFixed(joint[0]);

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
  dReal r,length;
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

  dGeomCapsuleGetParams(base.geom, &r, &length);
  dsDrawCylinder(dBodyGetPosition(base.body), dBodyGetRotation(base.body), length, r);

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
  //dsDrawSphere(dBodyGetPosition(bodyparts6[3].body),dBodyGetRotation(bodyparts6[3].body), 0.04);

}

// 位置センサの描画
void drawSensor()
{
 double R,G,B;
 dReal r = 0.04;
 R = 0/255;
 G = 153/255;
 B = 255/255;

 dsSetColor(R,G,B);
 dBodyGetRotation(sensor);
 dsDrawSphere(dBodyGetPosition(sensor), dBodyGetRotation(sensor), r);
}

// 目標位置の描画
void drawP()
{
 dReal tmpP[3];
 dMatrix3 tmpR;

 tmpP[0] = P[0];
 tmpP[1] = P[1];
 tmpP[2] = P[2];

 dsSetColor(1,0,0);

 dRSetIdentity(tmpR);
 dsDrawSphere(tmpP, tmpR, 0.06);
   //printf("P= %f %f %f \n",tmpP[0],tmpP[1],tmpP[2]);
}

/*** 制御 ***/
void Pcontrol()
{
  dReal k =  10.0, fMax = 1000.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < 7; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    if (z >=   M_PI) z -= 2.0 * M_PI;
    if (z <= - M_PI) z += 2.0 * M_PI;
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

// 逆運動学
void  inverseKinematics()
{
  double Px, Py, Pz;
  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
  double a2[3];
  double b2[3];

  double P5x = Px - (l[5] + l[6])*a[0];
  double P5y = Py - (l[5] + l[6])*a[1];
  double P5z = Pz - (l[5] + l[6])*a[2];

  printf("Target  Position: x=%6.2f y=%6.2f z=%6.2f \n", Px, Py, Pz);
  //printf("%6.2f\t%6.2f\t%6.2f\n", Px, Py, Pz);

  double tmpL  = sqrt(P5x * P5x + P5y * P5y);
  double P1P   = sqrt(P5x * P5x + P5y * P5y
   + (P5z - (l[0] + l[1])) * (P5z - (l[0] + l[1])));
  double Ca    = (l[2] * l[2] + P1P * P1P -l[3] * l[3])/(2 * l[2] * P1P);  // cosα

  double phi   = atan2(P5z - (l[0] + l[1]), tmpL);                      //φ
  double alpha = atan2(sqrt(1 - Ca * Ca), Ca);                         //α

  double Cb    = (l[2]*l[2] + l[3]*l[3] - P1P*P1P)/(2 * l[2] * l[3]);  //cosβ
  double beta  = atan2(sqrt(1- Cb * Cb), Cb);                          //β


  switch (ANSWER) { // ANSWERはキーボードからの入力で変更
    case 1:
    case 2:
    THETA[1] = atan2(P5x, P5y);
    THETA[2] = M_PI/2 - phi - alpha;
    THETA[3] = M_PI - beta; break;
    case 3:
    case 4:
    THETA[1] = atan2(P5x, P5y);
    THETA[2] = M_PI/2 - phi + alpha;
    THETA[3] = M_PI + beta; break;
    case 5:
    case 6:
    THETA[1] = atan2(P5x, P5y) + M_PI;
    THETA[2] = -(M_PI/2 - phi - alpha);
    THETA[3] = M_PI + beta; break;
    case 7:
    case 8:
    THETA[1] = atan2(P5x, P5y) + M_PI;
    THETA[2] = -(M_PI/2 - phi + alpha);
    THETA[3] = M_PI - beta; break;
  }

  a2[0] = cos(THETA[2]+THETA[3])*(a[0]*cos(THETA[1])+a[1]*sin(THETA[1])) - a[2]*sin(THETA[2]+THETA[3]);
  a2[1] = -a[0]*sin(THETA[1]) + a[1]*cos(THETA[1]);
  a2[2] = sin(THETA[2]+THETA[3])*(a[0]*cos(THETA[1])+a[1]*sin(THETA[1])) + a[2]*cos(THETA[2]+THETA[3]);
  b2[0] = cos(THETA[2]+THETA[3])*(b[0]*cos(THETA[1])+b[1]*sin(THETA[1])) - b[2]*sin(THETA[2]+THETA[3]);
  b2[1] = -b[0]*sin(THETA[1]) + b[1]*cos(THETA[1]);
  b2[2] = sin(THETA[2]+THETA[3])*(b[0]*cos(THETA[1])+b[1]*sin(THETA[1])) + b[2]*cos(THETA[2]+THETA[3]);

  switch (ANSWER) { // ANSWERはキーボードからの入力で変更
    case 1:
    case 3:
    case 5:
    case 7:
    THETA[4] = atan2(a2[1], a2[0]);
    break;
    case 2:
    case 4:
    case 6:
    case 8:
    THETA[4] = atan2(a2[1], a2[0]) + M_PI;
    break;
  }

  THETA[5] = atan2(cos(THETA[4]) * a2[0] + sin(THETA[4]) * a2[1], a2[2]);
  THETA[6] = atan2(sin(THETA[4]) * sin(THETA[5])*b2[0] - cos(THETA[4])*sin(THETA[5])*b2[1], b2[2]);
}

void yugan_a()
{
  a[0] = sin(T[0])*cos(T[1]);
  a[1] = sin(T[0])*sin(T[1]);
  a[2] = cos(T[0]);
}


void simLoop(int pause)
{
  yugan_a();
  inverseKinematics();
  printSensorPosition();
  Pcontrol();                                  // P制御
  dWorldStep(world, 0.01);                     // 動力学計算
  drawArm();                                   // ロボットの描画
  drawP();                                     // 目標位置の描画
  drawSensor();                                // 先端位置の描画
}

/*** キー入力関数 ***/
void command2(int cmd)
{
  switch (cmd) {
    case '1':  ANSWER = 1; break;    // 1キーを押すと姿勢１
    case '2':  ANSWER = 2; break;    // 2キーを押すと姿勢２
    case '3':  ANSWER = 3; break;    // 3キーを押すと姿勢３
    case '4':  ANSWER = 4; break;    // 4キーを押すと姿勢４
    case '5':  ANSWER = 5; break;    // 1キーを押すと姿勢１
    case '6':  ANSWER = 6; break;    // 2キーを押すと姿勢２
    case '7':  ANSWER = 7; break;    // 3キーを押すと姿勢３
    case '8':  ANSWER = 8; break;    // 4キーを押すと姿勢４
    case 'j':  P[0] += 0.01; break;   // jキーを押すと先端のx座標が増加
    case 'f':  P[0] -= 0.01; break;   // fキーを押すと先端のx座標が減少
    case 'k':  P[1] += 0.01; break;   // kキーを押すと先端のy座標が増加
    case 'd':  P[1] -= 0.01; break;   // dキーを押すと先端のy座標が減少
    case 'l':  P[2] += 0.01; break;   // lキーを押すと先端のz座標が増加
    case 's':  P[2] -= 0.01; break;   // sキーを押すと先端のz座標が減少
    case 'z':  T[0] += 0.01; break;   // zキーを押すと有顔ベクトルのθが増加
    case 'v':  T[0] -= 0.01; break;   // vキーを押すと有顔ベクトルのθが減少
    case 'x':  T[1] += 0.01; break;   // xキーを押すと有顔ベクトルのφが増加
    case 'c':  T[1] -= 0.01; break;   // cキーを押すと有顔ベクトルのφが減少
  }
}

/*** ドロースタッフの設定 ***/
void setDrawStuff()
{
  fn.version = DS_VERSION;                     // バージョン番号
  fn.start   = &start;                         // start関数
  fn.step    = &simLoop;                       // simLoop関数
  fn.command = &command2;                       // command関数
  fn.path_to_textures = "textures";
}

int main(int argc, char **argv)
{
  dInitODE(); //ＯＤＥの初期化
  setDrawStuff();
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  ground       = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定
  makeArm();                                      // アームの生成
  makeSensor();                                   // センサの生成

  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ
  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE(); //ＯＤＥの終了
  return 0;
}
