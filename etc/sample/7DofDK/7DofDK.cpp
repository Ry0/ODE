#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox      dsDrawBoxD
#endif
#define NUM 10                          // リンク数

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
dReal  THETA[NUM] = {0.000000,
                     0.401426,
                     -0.052360,
                     -0.3,
                     -0.349066,
                     0.628319,
                     0.000000,
                     0.418879,
                     1.343904,
                     -0.261799};             // 関節の目標角度[rad]
// dReal  THETA[NUM] = {0.0};
double P[3],a[3],b[3];                 // 先端の位置，有顔ベクトル(a,b)

dBodyID       sensor;        // センサ用のボディID
dJointID      sensor_joint;  // センサ固定用の関節

// センサの作成
void makeSensor()
{
  dMass mass;
  double sx = 0.0, sy = 0.0, sz = 2.645;  // センサの初期座標[m]
  double size = 0.01, weight = 0.00001; // センサのサイズ[m]と重量[kg]

  sensor = dBodyCreate(world);          // センサの生成
  dBodySetPosition(sensor,sx,sy,sz);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,weight,size,size,size);
  dBodySetMass(sensor, &mass);

  sensor_joint = dJointCreateFixed(world, 0); // 固定ジョイントの生成
  dJointAttach(sensor_joint, rlink[NUM-1].body, sensor); // 先端リンクと結合
  dJointSetFixed(sensor_joint);
}

// センサ位置の表示
void printSensorPosition()
{
  for (int i = 1; i < 10; ++i)
  {
    printf("THETA[%d] = %lf\n", i,THETA[i]);
  }
  double *pos = (double *) dBodyGetPosition(sensor);
  printf("P*: x=%5.3f y=%5.3f z=%5.3f \n",pos[0],pos[1],pos[2]);
  printf("P : x=%5.3f y=%5.3f z=%5.3f \n",P[0],P[1],P[2]);
}

// センサ姿勢(有顔ベクトル)の表示
void printSensorRotation()
{
  dReal a3[3],b3[3];
  dReal a0[3] = {0,0,1},b0[3] = {1,0,0};
  const dReal *R   = (const dReal *) dBodyGetRotation(sensor);

  dMultiply0((dReal *) a3, R, a0, 3, 3, 1);
  dMultiply0((dReal *) b3, R, b0, 3, 3, 1);
  // printf("a*: x=%5.2f y=%5.2f z=%5.2f",    a[0],a[1],a[2]);
  // printf("b*: x=%5.2f y=%5.2f z=%5.2f \n", b[0],b[1],b[2]);
  // printf("a : x=%5.2f y=%5.2f z=%5.2f",   a3[0],a3[1],a3[2]);
  // printf("b : x=%5.2f y=%5.2f z=%5.2f \n",b3[0],b3[1],b3[2]);
}

/*** ロボットアームの生成 ***/
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.005, 0.005, 0.16, 0.5375, 0.9725, 1.3075, 1.68, 2.19, 2.495, 2.595};  // 重心 z
  dReal length[NUM] = {0.01, 0.01, 0.32, 0.435, 0.435, 0.235, 0.51, 0.51, 0.1, 0.1};  // 長さ
  dReal weight[NUM] = {0.1, 0.1, 3.2, 4.35, 4.35, 2.35, 5.1, 5.1, 1, 1};  // 質量
  dReal r[NUM]      = {0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04};  // 半径
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.00, 0.00, 0.00, 0.32, 0.755, 1.19, 1.425, 1.935, 2.445, 2.545};  // 関節中心点 z
  dReal axis_x[NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};              // 関節回転軸 x
  dReal axis_y[NUM] = {0, 0, 1, 1, 1, 1, 1, 0, 1, 0};              // 関節回転軸 y
  dReal axis_z[NUM] = {1, 1, 0, 0, 0, 0, 0, 1, 0, 1};              // 関節回転軸 z

  // リンクの生成
  for (int i = 0; i < NUM; i++) {
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,weight[i],3,r[i],length[i]);
    dBodySetMass(rlink[i].body, &mass);
    rlink[i].geom = dCreateCapsule(space,r[i],length[i]);
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

  dJointSetHingeParam(joint[2],dParamLoStop, M_PI/2);
  dJointSetHingeParam(joint[2],dParamHiStop, M_PI/2);
  // dJointSetHingeParam(joint[3],dParamHikkStop, -M_PI/2);
  dJointSetHingeParam(joint[6],dParamLoStop, M_PI/2);
  dJointSetHingeParam(joint[6],dParamHiStop, M_PI/2);
}

/*** ロボットアームの描画 ***/
void drawArm()
{
	dReal r,length;

	for (int i = 0; i < NUM; i++ ) {       // カプセルの描画
		dGeomCapsuleGetParams(rlink[i].geom, &r,&length);
		dsDrawCylinder(dBodyGetPosition(rlink[i].body), dBodyGetRotation(rlink[i].body),length,r);
    dsSetColor(14.0/100.0, 51.0/100.0, 1.0);
	}
}

void drawSensor()
{
   dReal sides[] = {0.1,0.1,0.1};

   dsSetColor(1,0,0);
   dBodyGetRotation(sensor);
   dsDrawBox(dBodyGetPosition(sensor),dBodyGetRotation(sensor),sides);
}

/*** P制御 ***/
void Pcontrol()
{
  dReal k =  100.0, fMax = 10000.0;                   // 比例ゲイン，最大トルク

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
    case 'k': THETA[3] += M_PI/180; break;     // kキー
    case 'd': THETA[3] -= M_PI/180; break;     // dキー
    case 'l': THETA[4] += M_PI/180; break;     // lキー
    case 's': THETA[4] -= M_PI/180; break;     // sキー
    case 'z': THETA[5] += M_PI/180; break;     // jキー
    case 'x': THETA[5] -= M_PI/180; break;     // fキー
    case 'c': THETA[7] += M_PI/180; break;     // kキー
    case 'v': THETA[7] -= M_PI/180; break;     // dキー
    case 'b': THETA[8] += M_PI/180; break;     // lキー
    case 'n': THETA[8] -= M_PI/180; break;     // sキー
    case 'm': THETA[9] += M_PI/180; break;     // lキー
    case ',': THETA[9] -= M_PI/180; break;     // sキー
  }
}

void directKinematics()
{
  int x = 0, y = 1, z = 2;
  // double l[4] = { 0.10, 0.90, 1.00, 1.00};      // リンクの長さ[m]
  double angle[8];                              // 関節の角度[rad]

  angle[1] = -dJointGetHingeAngle(joint[1]);     // 第1関節角度の取得
  angle[2] = -dJointGetHingeAngle(joint[3]);     // 第2関節角度の取得
  std::cout << angle[2] << std::endl;
  angle[3] = -dJointGetHingeAngle(joint[4]);     // 第3関節角度の取得
  angle[4] = -dJointGetHingeAngle(joint[5]);     // 第1関節角度の取得
  angle[5] = -dJointGetHingeAngle(joint[7]);     // 第2関節角度の取得
  angle[6] = -dJointGetHingeAngle(joint[8]);     // 第3関節角度の取得
  angle[7] = -dJointGetHingeAngle(joint[9]);     // 第3関節角度の取得


  P[x] = (8*cos(angle[1]))/25 + (87*cos(angle[1])*cos(angle[2]))/200 + (51*cos(angle[4])*(cos(angle[1])*cos(angle[2])*sin(angle[3]) + cos(angle[1])*cos(angle[3])*sin(angle[2])))/50 - (47*cos(angle[4])*(cos(angle[1])*sin(angle[2])*sin(angle[3]) - cos(angle[1])*cos(angle[2])*cos(angle[3])))/200 + (sin(angle[6])*(sin(angle[1])*sin(angle[5]) - cos(angle[5])*(cos(angle[4])*(cos(angle[1])*sin(angle[2])*sin(angle[3]) - cos(angle[1])*cos(angle[2])*cos(angle[3])) + sin(angle[4])*(cos(angle[1])*cos(angle[2])*sin(angle[3]) + cos(angle[1])*cos(angle[3])*sin(angle[2])))))/5 - (47*sin(angle[4])*(cos(angle[1])*cos(angle[2])*sin(angle[3]) + cos(angle[1])*cos(angle[3])*sin(angle[2])))/200 - (51*sin(angle[4])*(cos(angle[1])*sin(angle[2])*sin(angle[3]) - cos(angle[1])*cos(angle[2])*cos(angle[3])))/50 + (cos(angle[6])*(cos(angle[4])*(cos(angle[1])*cos(angle[2])*sin(angle[3]) + cos(angle[1])*cos(angle[3])*sin(angle[2])) - sin(angle[4])*(cos(angle[1])*sin(angle[2])*sin(angle[3]) - cos(angle[1])*cos(angle[2])*cos(angle[3]))))/5 - (87*cos(angle[1])*sin(angle[2])*sin(angle[3]))/200 + (87*cos(angle[1])*cos(angle[2])*cos(angle[3]))/200;
  P[y] = -((8*sin(angle[1]))/25 + (87*cos(angle[2])*sin(angle[1]))/200 + (51*cos(angle[4])*(cos(angle[2])*sin(angle[1])*sin(angle[3]) + cos(angle[3])*sin(angle[1])*sin(angle[2])))/50 - (47*cos(angle[4])*(sin(angle[1])*sin(angle[2])*sin(angle[3]) - cos(angle[2])*cos(angle[3])*sin(angle[1])))/200 - (47*sin(angle[4])*(cos(angle[2])*sin(angle[1])*sin(angle[3]) + cos(angle[3])*sin(angle[1])*sin(angle[2])))/200 - (51*sin(angle[4])*(sin(angle[1])*sin(angle[2])*sin(angle[3]) - cos(angle[2])*cos(angle[3])*sin(angle[1])))/50 - (sin(angle[6])*(cos(angle[1])*sin(angle[5]) + cos(angle[5])*(cos(angle[4])*(sin(angle[1])*sin(angle[2])*sin(angle[3]) - cos(angle[2])*cos(angle[3])*sin(angle[1])) + sin(angle[4])*(cos(angle[2])*sin(angle[1])*sin(angle[3]) + cos(angle[3])*sin(angle[1])*sin(angle[2])))))/5 + (cos(angle[6])*(cos(angle[4])*(cos(angle[2])*sin(angle[1])*sin(angle[3]) + cos(angle[3])*sin(angle[1])*sin(angle[2])) - sin(angle[4])*(sin(angle[1])*sin(angle[2])*sin(angle[3]) - cos(angle[2])*cos(angle[3])*sin(angle[1]))))/5 - (87*sin(angle[1])*sin(angle[2])*sin(angle[3]))/200 + (87*cos(angle[2])*cos(angle[3])*sin(angle[1]))/200);
  P[z] = (87*sin(angle[2]))/200 + (87*cos(angle[2])*sin(angle[3]))/200 + (87*cos(angle[3])*sin(angle[2]))/200 - (cos(angle[6])*(cos(angle[4])*(cos(angle[2])*cos(angle[3]) - sin(angle[2])*sin(angle[3])) - sin(angle[4])*(cos(angle[2])*sin(angle[3]) + cos(angle[3])*sin(angle[2]))))/5 + (47*cos(angle[4])*(cos(angle[2])*sin(angle[3]) + cos(angle[3])*sin(angle[2])))/200 - (51*cos(angle[4])*(cos(angle[2])*cos(angle[3]) - sin(angle[2])*sin(angle[3])))/50 + (51*sin(angle[4])*(cos(angle[2])*sin(angle[3]) + cos(angle[3])*sin(angle[2])))/50 + (47*sin(angle[4])*(cos(angle[2])*cos(angle[3]) - sin(angle[2])*sin(angle[3])))/200 + (cos(angle[5])*sin(angle[6])*(cos(angle[4])*(cos(angle[2])*sin(angle[3]) + cos(angle[3])*sin(angle[2])) + sin(angle[4])*(cos(angle[2])*cos(angle[3]) - sin(angle[2])*sin(angle[3]))))/5;

  // 有顔ベクトル
  a[x] =  cos(angle[1]) * sin(angle[2] + angle[3]);    // 主軸:x座標
  a[y] =  sin(angle[1]) * sin(angle[2] + angle[3]);    // 主軸:y座標
  a[z] =                  cos(angle[2] + angle[3]);    // 主軸:z座標
  b[x] =  cos(angle[1]) * cos(angle[2] + angle[3]);    // 副軸:x座標
  b[y] =  sin(angle[1]) * cos(angle[2] + angle[3]);    // 副軸:y座標
  b[z] =                - sin(angle[2] + angle[3]);    // 副軸:z座標
  // printf("P: x=%5.2f y=%5.2f z=%5.2f,",  P[0],P[1],P[2]);
  // printf("a: x=%5.2f y=%5.2f z=%5.2f,",  a[0],a[1],a[2]);
  // printf("b: x=%5.2f y=%5.2f z=%5.2f \n",b[0],b[1],b[2]);
}


/*** シミュレーションループ ***/
void simLoop(int pause)
{
  Pcontrol();                                  // P制御
  directKinematics();
  printSensorPosition();
  printSensorRotation();
	dWorldStep(world, 0.01);                     // 動力学計算
  drawArm();                                   // ロボットの描画
	drawSensor();
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
  dInitODE(); // ODEの初期化
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
  dCloseODE();
  return 0; // ODEの終了
}
