#include "area_struct.h"
#include "arm_struct.h"
#include "control.h"
#include "obstacle_struct.h"

// #define PLOT
#ifdef PLOT
#include <deque>
#include "pipestream.h"
using namespace ode_utils;
deque<PlotData> plotData;
ps::pipestream *gnuplot;

double times = 0;
int cnt = 0;
#endif

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ

dJointID      joint[7];                // ジョイントのID番号
dJointID      bodyjoint1[8], bodyjoint2[13], bodyjoint3[4], bodyjoint4[4], bodyjoint5, bodyjoint6[4]; // ロボットアーム用のジョイント郡
dJointID      boxjoint;
dJointID      sensor_joint;            // センサ固定用の関節
dBodyID       sensor;                  // センサ用のボディID]

dsFunctions   fn;                      // ドロースタッフの描画関数

MyObject base, bodyparts1[9], bodyparts2[14], bodyparts3[5], bodyparts4[5], bodyparts5[2], bodyparts6[5]; // ロボットアーム用のパーツ群
MyObject boxparts;

dReal  THETA[7] = {0.0};               // 関節の目標角度[rad]
dReal min_angle[6] = {-170.0*M_PI/180.0, -80.0*M_PI/180.0, -90.0*M_PI/180.0, -190.0*M_PI/180.0, -120.0*M_PI/180.0, -360.0*M_PI/180.0}; // 各関節の最小角度[rad]
dReal max_angle[6] = {170.0*M_PI/180.0, 135.0*M_PI/180.0, 155.0*M_PI/180.0, 190.0*M_PI/180.0, 120.0*M_PI/180.0, 360.0*M_PI/180.0};     // 各関節の最小角度[rad]

int ANSWER = 1;                        // 逆運動学の解
int i = 0;                             // simLoopのループカウント用変数
int data_num = 0;                      // 経路データを読み込んだ時のデータ点数格納用変数

dReal P[3] = {0.35, 0.0, 0.20};        // 先端の位置

// 有顔ベクトル(a,b)
dReal a[3];
dReal b[3] = {0.0, 0.0, 1.0};
dReal T[2] = {M_PI, 0.0};
dReal l[7] = {0.20, 0.145, 0.33, 0.34, 0.34, 0.073, 0.18+0.04}; // リンクの長さ[m]

vector< POINT > pathdata;             // 経路データのxyz座標
dReal StartP[3] = {0.0};              // 計画経路のスタート地点
dReal GoalP[3] = {0.0};               // 計画経路のゴール地点

static int ModeSelector; // 自由操作モードと経路データ読み込みモード

/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = {    1.5f, 0.65f, 0.4f};         // 視点[m]
  float hpr[3] = { -160.0f, 4.5f, 0.0f};          // 視線[°]
  dsSetViewpoint(xyz, hpr);                       // 視点と視線の設定
}

// POINT p = {0.35, 0.35, 0.20};          //?わっからーん

void simLoop(int pause)
{
  // P[0] = 0.4;
  // P[1] = 0.1+0.16*pow(sin(0.01*i),3);
  // P[2] = 0.3 + 0.13*cos(0.01*i) - 0.05*cos(2*0.01*i) - 0.02*cos(3*0.01*i) - 0.01*cos(4*0.01*i);

  std::cout << "step: " << i << std::endl;

  #ifdef PLOT
  if(!pause){
    PlotData d = { times, P[0], P[1], P[2] }; // x, y, z
    plotData.push_back(d);

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
  if (!pause){
    dWorldStep( world, 0.01 );
    times += 0.01;
    ++cnt;
  }
  #endif

  yugan_a();
  inverseKinematics();
  printEndArmPosition();
  Pcontrol();                                      // P制御
  dWorldStep(world, 0.01);                         // 動力学計算
  drawArmCenter();                                 // ロボットの描画
  drawArmSide();                                   // ロボットの描画
  drawGripper_start();
  drawGripper();
  drawGripper_edge();
  drawBase();
  drawSensor();                                   // 先端位置の描画

  if(ModeSelector == 0){
    drawP();                                      // 目標位置の描画
  } else if(ModeSelector == 1) {
    printPosition(pathdata, i);
    drawBox();
  }
  i++;
}

/*** キー入力関数 ***/
void command(int cmd)
{
  switch (cmd) {
    case '1':  ANSWER = 1; break;     // 1キーを押すと姿勢１
    case '2':  ANSWER = 2; break;     // 2キーを押すと姿勢２
    case '3':  ANSWER = 3; break;     // 3キーを押すと姿勢３
    case '4':  ANSWER = 4; break;     // 4キーを押すと姿勢４
    case '5':  ANSWER = 5; break;     // 1キーを押すと姿勢１
    case '6':  ANSWER = 6; break;     // 2キーを押すと姿勢２
    case '7':  ANSWER = 7; break;     // 3キーを押すと姿勢３
    case '8':  ANSWER = 8; break;     // 4キーを押すと姿勢４
    case 'j':  P[0] += 0.01; break;   // jキーを押すと先端のx座標が増加
    case 'f':  P[0] -= 0.01; break;   // fキーを押すと先端のx座標が減少
    case 'k':  P[1] += 0.01; break;   // kキーを押すと先端のy座標が増加
    case 'd':  P[1] -= 0.01; break;   // dキーを押すと先端のy座標が減少
    case 'l':  P[2] += 0.01; break;   // lキーを押すと先端のz座標が増加
    case 's':  P[2] -= 0.01; break;   // sキーを押すと先端のz座標が減少
    case 'z':  T[0] += 0.05; break;   // zキーを押すと有顔ベクトルのθが増加
    case 'v':  T[0] -= 0.05; break;   // vキーを押すと有顔ベクトルのθが減少
    case 'x':  T[1] += 0.05; break;   // xキーを押すと有顔ベクトルのφが増加
    case 'c':  T[1] -= 0.05; break;   // cキーを押すと有顔ベクトルのφが減少
    case 'r':                         // rキーを押すと初期姿勢に戻る
      P[0] = 0.35;
      P[1] = 0.0;
      P[2] = 0.20;
      T[0] = M_PI;
      T[1] = 0.0;
      break;
  }
}


/*** ドロースタッフの設定 ***/
void setDrawStuff()
{
  fn.version = DS_VERSION;                     // バージョン番号
  fn.start   = &start;                         // start関数
  fn.step    = &simLoop;                       // simLoop関数
  fn.command = &command;                      // command関数
  fn.path_to_textures = "textures";
}


int main(int argc, char* argv[])
{
  dInitODE(); //ＯＤＥの初期化
  setDrawStuff();

  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  ground       = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定

  makeBase();
  makeArm();                                      // アームの生成
  makeSensor();                                   // センサの生成

  ModeSelector = input_arg(argc, argv);
  if(ModeSelector == 1){
    makeBox();
  }else if(ModeSelector == 2){
    return -1;
  }

  if(YesorNo() == false){
    return -1;
  }


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
