#include "area_struct.h"
#include "arm_struct.h"

extern dWorldID      world;                   // 動力学計算用のワールド
extern dSpaceID      space;                   // 衝突検出用のスペース
extern dGeomID       ground;                  // 地面のジオメトリID番号
extern dJointGroupID contactgroup;            // 接触点グループ
extern dJointID      joint[NUM];              // ジョイントのID番号
extern dsFunctions   fn;                      // ドロースタッフの描画関数

extern MyObject rlink[NUM];                   // リンク

extern dBodyID       sensor;                  // センサ用のボディID
extern dJointID      sensor_joint;            // センサ固定用の関節
extern int ANSWER;              // 逆運動学の解
extern int i;                             // simLoopのループカウント用変数
extern int data_num;                      // 経路データを読み込んだ時のデータ点数格納用変数

extern dReal P[3];             // 先端の位置
// 有顔ベクトル(a,b)
extern dReal a[3];
extern dReal b[3];
extern dReal T[2];
extern dReal THETA[NUM];  // 関節の目標角度[rad]
extern dReal tmpTHETA3, tmpTHETA5;
extern dReal min_angle[NUM]; // 各関節の最小角度[rad]
extern dReal max_angle[NUM]; // 各関節の最大角度[rad]

extern dReal l[NUM];   // リンクの長さ[m]


// センサの作成
void makeSensor()
{
  dMass mass;
  double sx = 0.0, sy = 0.0, sz = 2.845;  // センサの初期座標[m]
  double size = 0.05, weight = 0.00001; // センサのサイズ[m]と重量[kg]

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
  printf("手先センサーからの値 : x=%5.3f y=%5.3f z=%5.3f \n",pos[0],pos[1],pos[2]);
  printf("運動学から導出した値 : x=%5.3f y=%5.3f z=%5.3f \n",P[0],P[1],P[2]+0.2);
  printf("\n\n");
}


/*** ロボットアームの生成 ***/
void makeArm()
{
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.05, 0.15, 0.36, 0.7375, 1.1725, 1.5075, 1.88, 2.39, 2.695, 2.795};  // 重心 z
  dReal length[NUM] = {0.10, 0.10, 0.32, 0.435, 0.435, 0.235, 0.51, 0.51, 0.10, 0.10};  // 長さ
  dReal weight[NUM] = {1.0, 1.0, 3.2, 4.35, 4.35, 2.35, 5.1, 5.1, 1.0, 1.0};  // 質量
  dReal r[NUM]      = {0.08, 0.08, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04};  // 半径
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.00, 0.10, 0.20, 0.52, 0.955, 1.39, 1.625, 2.135, 2.645, 2.745};  // 関節中心点 z
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

  // 2と6は90度で固定
  dJointSetHingeParam(joint[2],dParamLoStop, M_PI/2);
  dJointSetHingeParam(joint[2],dParamHiStop, M_PI/2);
  dJointSetHingeParam(joint[6],dParamLoStop, M_PI/2);
  dJointSetHingeParam(joint[6],dParamHiStop, M_PI/2);
  // for (int i = 1; i < 10; i++) {
  //   dJointSetHingeParam(joint[i],dParamLoStop, min_angle[i-1]);
  //   dJointSetHingeParam(joint[i],dParamHiStop, max_angle[i-1]);
  // }
}

/*** ロボットアームの描画 ***/
void drawArm()
{
  dReal r,length;

  for (int i = 0; i < NUM; i++ ) {       // カプセルの描画
    dGeomCapsuleGetParams(rlink[i].geom, &r,&length);
    if (i != NUM -1 && i != 0 && i != 1){
      dsDrawCapsule(dBodyGetPosition(rlink[i].body),
      dBodyGetRotation(rlink[i].body),length,r);
      if(i==2 || i == 3){
        dsSetColor(1,0,0);
      }else{
        dsSetColor(31.0/255.0, 80.0/255.0, 1);
      }
    }else{
      dsSetColor(31.0/255.0, 80.0/255.0, 1);
      dsDrawCylinder(dBodyGetPosition(rlink[i].body),dBodyGetRotation(rlink[i].body),length,r);
    }
  }
}

// 位置センサの描画
void drawSensor()
{
 double R,G,B;
 dReal sides[] = {0.05,0.05,0.04};
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
   // printf("P= %f %f %f \n",tmpP[0],tmpP[1],tmpP[2]);
}

// 目標位置の描画
void drawP5()
{
  dReal tmpP[3];
  dMatrix3 tmpR;

  double P5x = P[0] - (l[8] + l[9])*a[0];
  double P5y = P[1] - (l[8] + l[9])*a[1];
  double P5z = P[2] - (l[8] + l[9])*a[2];


  tmpP[0] = P5x;
  tmpP[1] = P5y;
  tmpP[2] = P5z;

  dsSetColor(31.0/255.0, 80.0/255.0, 1);

  dRSetIdentity(tmpR);
  dsDrawSphere(tmpP, tmpR, 0.06);
   //printf("P= %f %f %f \n",tmpP[0],tmpP[1],tmpP[2]);
}
