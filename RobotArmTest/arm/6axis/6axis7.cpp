#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <signal.h>
#include <pthread.h>

#ifdef dDOUBLE
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
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

dBodyID       sensor;                  // センサ用のボディID
dJointID      sensor_joint;            // センサ固定用の関節
int           ANSWER = 1;              // 逆運動学の解
int i,j = 0;

dReal P[3] = {1.0, 0.5, 1.5};             // 先端の位置
// 有顔ベクトル(a,b)
dReal a[3];//?わっからーん
dReal b[3] = {0.0, 0.0, 1.0};//?わっからーん
dReal T[2] = {M_PI, 0.0};
dReal THETA[NUM] = {0.0};  // 関節の目標角度[rad]
dReal l[NUM] = {0.10, 0.90, 1.00, 1.0, 0.50, 0.25, 0.25};   // リンクの長さ[m]

#ifdef RECORD
typedef struct {
  double x;
  double y;
  double z;
} state;
state target_state;
pthread_mutex_t mtx;
#endif //RECORD

// センサの作成
void makeSensor()
{
  dMass mass;
  double sx = 0.0, sy = 0.0, sz = 3.5;  // センサの初期座標[m]
  double size = 0.10, weight = 0.00001; // センサのサイズ[m]と重量[kg]

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
  double *pos = (double *) dBodyGetPosition(sensor);
  // printf("Current Position: x=%6.2f y=%6.2f z=%6.2f \n",pos[0],pos[1],pos[2]);
  printf("%6.2f\t%6.2f\t%6.2f\t",pos[0],pos[1],pos[2]);
  // printf("P : x=%5.2f y=%5.2f z=%5.2f \n",P[0],P[1],P[2]);
}

/*** ロボットアームの生成 ***/
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.05, 0.55, 1.5, 2.25, 2.75, 3.125, 3.375};  // 重心 z
  dReal length[NUM] = {0.10, 0.90, 1.00, 0.50, 0.50, 0.25, 0.25};  // 長さ
  dReal weight[NUM] = {9.00, 2.00, 2.00, 1.00, 1.00, 0.50, 0.50};  // 質量
  dReal r[NUM]      = {0.20, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04};  // 半径
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.00, 0.10, 1.00, 2.00, 2.50, 3.00, 3.25};  // 関節中心点 z
  dReal axis_x[NUM] = {0, 0, 0, 0, 0, 0, 0};              // 関節回転軸 x
  dReal axis_y[NUM] = {0, 0, 1, 1, 0, 1, 0};              // 関節回転軸 y
  dReal axis_z[NUM] = {1, 1, 0, 0, 1, 0, 1};              // 関節回転軸 z

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
}

/*** ロボットアームの描画 ***/
void drawArm()
{
  dReal r,length;

  for (int i = 0; i < NUM; i++ ) {       // カプセルの描画
    dGeomCapsuleGetParams(rlink[i].geom, &r,&length);
    if (i != NUM -1 && i != 0)
     dsDrawCapsule(dBodyGetPosition(rlink[i].body),
      dBodyGetRotation(rlink[i].body),length,r);
   else
     dsDrawCylinder(dBodyGetPosition(rlink[i].body),dBodyGetRotation(rlink[i].body),length,r);
 }
}

// 位置センサの描画
void drawSensor()
{
 double R,G,B;
 dReal sides[] = {0.10,0.10,0.04};
 R = 0/255;
 G = 153/255;
 B = 255/255;

 dsSetColor(R,G,B);
 dBodyGetRotation(sensor);
 dsDrawBox(dBodyGetPosition(sensor),dBodyGetRotation(sensor),sides);
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

// 目標位置の描画
void drawP5()
{
  dReal tmpP[3];
  dMatrix3 tmpR;

  double P5x = P[0] - (l[5] + l[6])*a[0];
  double P5y = P[1] - (l[5] + l[6])*a[1];
  double P5z = P[2] - (l[5] + l[6])*a[2];


  tmpP[0] = P5x;
  tmpP[1] = P5y;
  tmpP[2] = P5z;

  dsSetColor(0,1,0);

  dRSetIdentity(tmpR);
  dsDrawSphere(tmpP, tmpR, 0.06);
   //printf("P= %f %f %f \n",tmpP[0],tmpP[1],tmpP[2]);
}

/*** 制御 ***/
void Pcontrol()
{
  dReal k =  10.0, fMax = 1000.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < NUM; j++) {
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
  float xyz[3] = {    3.0f, 1.3f, 0.8f};          // 視点[m]
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

  // printf("Target  Position: x=%6.2f y=%6.2f z=%6.2f \n", Px, Py, Pz);
  printf("%6.2f\t%6.2f\t%6.2f\n", Px, Py, Pz);

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
      THETA[1] = atan2(P5y, P5x);
      THETA[2] = M_PI/2 - phi - alpha;
      THETA[3] = M_PI - beta; break;
    case 3:
    case 4:
      THETA[1] = atan2(P5y, P5x);
      THETA[2] = M_PI/2 - phi + alpha;
      THETA[3] = M_PI + beta; break;
    case 5:
    case 6:
      THETA[1] = atan2(P5y, P5x) + M_PI;
      THETA[2] = -(M_PI/2 - phi - alpha);
      THETA[3] = M_PI + beta; break;
    case 7:
    case 8:
      THETA[1] = atan2(P5y, P5x) + M_PI;
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

/*** シミュレーションループ ***/
void simLoop(int pause)
{

  // P[1] = 0.5*cos(0.01*i);
  // P[2] = 1.5+0.5*sin(0.01*i);
  P[0] = 1.0 + 0.5*cos(0.01*i);
  P[1] = 0.5*sin(0.01*i)*cos(0.01*j);
  P[2] = 1.5+0.5*sin(0.01*i)*sin(0.01*j);
  pthread_mutex_lock(&mtx);
  state.x = P[0]; state.y = P[1]; state.z = P[2];
  pthread_mutex_unlock(&mtx);

  yugan_a();
  inverseKinematics();
  printSensorPosition();
  Pcontrol();                                  // P制御
  dWorldStep(world, 0.01);                     // 動力学計算
  drawArm();                                   // ロボットの描画
  drawP();                                     // 目標位置の描画
  drawSensor();                                // 先端位置の描画
  i++;
  j++;
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
    case 'j':  P[0] += 0.1; break;   // jキーを押すと先端のx座標が増加
    case 'f':  P[0] -= 0.1; break;   // fキーを押すと先端のx座標が減少
    case 'k':  P[1] += 0.1; break;   // kキーを押すと先端のy座標が増加
    case 'd':  P[1] -= 0.1; break;   // dキーを押すと先端のy座標が減少
    case 'l':  P[2] += 0.1; break;   // lキーを押すと先端のz座標が増加
    case 's':  P[2] -= 0.1; break;   // sキーを押すと先端のz座標が減少
    case 'z':  T[0] += 0.1; break;   // zキーを押すと有顔ベクトルのθが増加
    case 'v':  T[0] -= 0.1; break;   // vキーを押すと有顔ベクトルのθが減少
    case 'x':  T[1] += 0.1; break;   // xキーを押すと有顔ベクトルのφが増加
    case 'c':  T[1] -= 0.1; break;   // cキーを押すと有顔ベクトルのφが減少
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


void *RecordThread(void *arg)
{
  record_pos.open("pos.dat", ios::out);
  if(! record_pos.is_open()){exit(-1);}

  while(1){
    pthread_testcancel();
    pthread_mutex_lock(&robot_pos_mtx);
    record_pos << simu_state.x << " " << simu_state.y << " " << simu_state.theta << endl;
    record_out << simu_out.r << " " << simu_out.l << " " << endl;
    pthread_mutex_unlock(&robot_pos_mtx);
    for(int i=0; i < 10; i++){
      usleep(1000);
    }
  }
}


int main(int argc, char *argv[])
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

#ifdef RECORD
  void *res;
  pthread_t record_thread_t;
  int s = pthread_create(&record_thread_t, NULL, RecordThread, NULL);
  if(s != 0){ exit(1); }else{ cout << "Record Thread create success" << endl; }
#endif

  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ

#ifdef RECORD
  s = pthread_cancel(record_thread_t);
  s = pthread_join(record_thread_t, &res);
  if(res == PTHREAD_CANCELED){ cout << "Record thread was canceled" << endl;}
#endif

  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE(); //ＯＤＥの終了
  return 0;
}
