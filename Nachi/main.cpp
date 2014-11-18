#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

//#define PLOT

#include "arm_struct.h"
#include "draw_arms.h"

#ifdef PLOT
using namespace std;
using namespace ode_utils;

deque<PlotData> plotData;

double times = 0;
int cnt = 0;
ps::pipestream *gnuplot;
#endif

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointID      joint[7];              // ジョイントのID番号
dJointID      bodyjoint1[8], bodyjoint2[13], bodyjoint3, bodyjoint4[4], bodyjoint5, bodyjoint6[4];
dJointID      boxjoint;
dBodyID       sensor;                  // センサ用のボディID
dJointID      sensor_joint;            // センサ固定用の関節
dsFunctions   fn;                      // ドロースタッフの描画関数


MyObject base, bodyparts1[9], bodyparts2[14], bodyparts3[2], bodyparts4[5], bodyparts5[2], bodyparts6[5];
MyObject boxparts;
dReal  THETA[7] = {0.0};             // 関節の目標角度[rad]

int  ANSWER = 1;              // 逆運動学の解
int  i,j = 0;
int data_num = 0;

dReal P[3] = {0.35, 0.35, 0.20};             // 先端の位置

// 有顔ベクトル(a,b)
dReal a[3];//?わっからーん
dReal b[3] = {0.0, 0.0, 1.0};//?わっからーん
dReal T[2] = {M_PI, 0.0};
dReal l[7] = {0.20, 0.145, 0.33, 0.34, 0.34, 0.073, 0.0};   // リンクの長さ[m]


void makeBox()
{
  dMass mass;                                    // 質量パラメータ

  dReal  box_mass  = 0.02;// 質量
  dReal  box_x_length = 0.1;
  dReal  box_y_length = 0.1;
  dReal  box_z_length = 0.9;

  dReal  box_start_x = 0.35;// 重心 y
  dReal  box_start_y = 0.05;// 重心 y
  dReal  box_start_z = 0.45;// 重心 z

  boxparts.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, box_mass, box_x_length, box_y_length, box_z_length);
  dBodySetMass(boxparts.body, &mass);

  boxparts.geom = dCreateBox(space, box_x_length, box_y_length, box_z_length);
  dGeomSetBody(boxparts.geom, boxparts.body);
  dBodySetPosition(boxparts.body, box_start_x, box_start_y, box_start_z);

  //パーツの合体
  boxjoint = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(boxjoint, boxparts.body, 0);
  dJointSetFixed(boxjoint);


}


void drawBox()
{
  dReal box_length[] = {0.1, 0.1, 0.9};

  double R = 0.0/255.0;
  double G = 191.0/255.0;
  double B = 255.0/255.0;
  dsSetColor(R,G,B);
  //dsSetColorAlpha(R, G, B, 0.5);

  dsDrawBox(dBodyGetPosition(boxparts.body), dBodyGetRotation(boxparts.body), box_length);

  dReal tmpP1[3],tmpP2[3];
  dMatrix3 tmpR1,tmpR2;

  tmpP1[0] = 0.35;
  tmpP1[1] = 0.35;
  tmpP1[2] = 0.40;

  tmpP2[0] = 0.20;
  tmpP2[1] = -0.35;
  tmpP2[2] = 0.10;

  double R2 = 255.0/255.0;
  double G2 = 20.0/255.0;
  double B2 = 147.0/255.0;

  dsSetColor(R2,G2,B2);

  dRSetIdentity(tmpR1);
  dsDrawSphere(tmpP1, tmpR1, 0.02);
  dRSetIdentity(tmpR2);
  dsDrawSphere(tmpP2, tmpR2, 0.02);

}


typedef struct {
  double x;
  double y;
  double z;
} POINT;
std::vector< POINT > vobstacle;


int CountNumbersOfTextLines(std::string fileName){
  int i = 0;
  std::ifstream ifs(fileName);// ファイルを開く。

  if(ifs){// ファイルのオープンに成功していれば、これは file は true を返す。
    std::string line;// 1行ごとの文字列を格納するための string オブジェクト。
    while( true ){
      getline( ifs, line );// 1行読み取る。
      if( ifs.eof() ){// ファイルが終端まで来たら、break 文を実行して while 文を抜ける。
        break;
      }
      i++;// 1行読み取ったので、インクリメントする。
    }
  }

  return i;// カウントした行数を返す。
}


void Input_Data(std::string fileName){

  std::ifstream input(fileName.c_str());
  data_num = CountNumbersOfTextLines(fileName);
  vobstacle.resize(data_num);
  printf("ばかばか\n");
  for (int i = 0; i < data_num; ++i){
    input >> vobstacle[i].x >> vobstacle[i].y >> vobstacle[i].z;
  }

  input.close();

}

/*** 制御 ***/
void Pcontrol()
{
  dReal k =  20.0, fMax = 200.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < 7; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    if (z >=   M_PI) z -= 2.0 * M_PI;
    if (z <= - M_PI) z += 2.0 * M_PI;
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax, fMax); // トルクの設定
  }
}


/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = {    1.5f, 0.65f, 0.4f};          // 視点[m]
  float hpr[3] = { -160.0f, 4.5f, 0.0f};          // 視線[°]
  dsSetViewpoint(xyz, hpr);                       // 視点と視線の設定
}


void  inverseKinematics()
{
  double Px, Py, Pz;
  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
  double a2[3];
  double b2[3];

  double P5x = Px - (l[5] + l[6])*a[0];
  double P5y = Py - (l[5] + l[6])*a[1];
  double P5z = Pz - (l[5] + l[6])*a[2];

  printf("Target  Position: x=%7.3f y=%7.3f z=%7.3f \n", Px, Py, Pz);

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
  printf("\nJoint   Angle   : 1=%7.2f 2=%7.2f 3=%7.2f \n",THETA[1]*180/M_PI,THETA[2]*180/M_PI,THETA[3]*180/M_PI);
  printf("                  4=%7.2f 5=%7.2f 6=%7.2f [deg]\n\n",THETA[4]*180/M_PI,THETA[5]*180/M_PI,THETA[6]*180/M_PI);
}


void yugan_a()
{
  a[0] = sin(T[0])*cos(T[1]);
  a[1] = sin(T[0])*sin(T[1]);
  a[2] = cos(T[0]);
}


void simLoop(int pause)
{
  // P[0] = 0.4;
  // P[1] = 0.1+0.16*pow(sin(0.01*i),3);
  // P[2] = 0.3 + 0.13*cos(0.01*i) - 0.05*cos(2*0.01*i) - 0.02*cos(3*0.01*i) - 0.01*cos(4*0.01*i);
  //障害物かわすようのコメントアウト
  // if((i/data_num)%2 == 0){
  //   P[0] = vobstacle[i%data_num].x;
  //   P[1] = vobstacle[i%data_num].y;
  //   P[2] = vobstacle[i%data_num].z;
  // }else{
  //   P[0] = vobstacle[data_num-1-i%data_num].x;
  //   P[1] = vobstacle[data_num-1-i%data_num].y;
  //   P[2] = vobstacle[data_num-1-i%data_num].z;
  // }

  //cout << "step: " << i << endl;
  // if(i>=data_num){
  //   P[0] = vobstacle[data_num-1].x;
  //   P[1] = vobstacle[data_num-1].y;
  //   P[2] = vobstacle[data_num-1].z;
  // }

  #ifdef PLOT
  if(!pause){
    PlotData d = { times, P[0], P[1], P[2] }; // x, y, z
    plotData.push_back( d );

    if (cnt%10 == 0) {
      *gnuplot << "splot \"./data/cube.dat\" using 1:2:3 with lines lw 5 lt rgb \"#696969\" title \"Obstacle\", \\\n '-' t 'Trajectory' with points pt 7 ps 1 lt rgb \"#F92500\"" << ps::endl;
      deque<PlotData>::iterator it = plotData.begin();
      while (it != plotData.end() ) {
        *gnuplot << (*it).x << " " << (*it).y << " " << (*it).z << ps::endl;
        ++it;
      }
      *gnuplot << "e" << ps::endl;
    }

    while (plotData.size() > 600) {
      plotData.pop_front();
    }
  }

  // Ctl+p が押されたらifに入らない
  if (!pause)
  {
    dWorldStep( world, 0.01 );
    times += 0.01;
    ++cnt;
  }
  #endif

  yugan_a();
  inverseKinematics();
  printEndArmPosition();
  Pcontrol();                                  // P制御
  dWorldStep(world, 0.01);                     // 動力学計算
  drawArmCenter();                                   // ロボットの描画
  drawArmSide();                                   // ロボットの描画
  drawGripper_start();
  // drawGripper();
  // drawGripper_edge();
  drawBase();
  drawP();                                     // 目標位置の描画
  // drawSensor();                                // 先端位置の描画
  //drawBox();

  i++;
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
  Input_Data("./data/data.dat");
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  ground       = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定
  makeBase();
  makeArm();                                      // アームの生成
  makeSensor();                                   // センサの生成

  //makeBox();

#ifdef PLOT
  gnuplot = new ps::pipestream( "gnuplot -geometry 480x480" );
  *gnuplot << "set ticslevel 0"<<ps::endl;
  *gnuplot << "set view 70, 110, 1, 1"<<ps::endl;
  *gnuplot << "set view equal xyz"<<ps::endl;
  *gnuplot << "set xrange[-0.4:0.4]"<<ps::endl;
  *gnuplot << "set yrange[-0.4:0.4]"<<ps::endl;
  *gnuplot << "set zrange[0:1]"<<ps::endl;
  *gnuplot << "set xtics 0.4"<<ps::endl;
  *gnuplot << "set ytics 0.4"<<ps::endl;
  *gnuplot << "set ztics 0.4"<<ps::endl;
  *gnuplot << "set mxtics 0.2"<<ps::endl;
  *gnuplot << "set mytics 0.2"<<ps::endl;
  *gnuplot << "set mztics 0.2"<<ps::endl;
#endif

  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ
  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE(); //ODEの終了
  return 0;
}
