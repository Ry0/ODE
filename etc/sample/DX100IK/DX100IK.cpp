#include "area_struct.h"
#include "arm_struct.h"
#include "control.h"

#define PLOT
#ifdef PLOT
#include <deque>
#include "pipestream.h"

using namespace ode_utils;

struct PlotData {
  double t;
  double x;
  double y;
  double z;
};
deque<PlotData> plotData;

double times = 0;
ps::pipestream *gnuplot;
#endif

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointID      joint[NUM];              // ジョイントのID番号
dsFunctions   fn;                      // ドロースタッフの描画関数



MyObject rlink[NUM];                   // リンク

dBodyID       sensor;                  // センサ用のボディID
dJointID      sensor_joint;            // センサ固定用の関節
int ANSWER = 1;              // 逆運動学の解
int i = 0;                             // simLoopのループカウント用変数
int data_num = 0;                      // 経路データを読み込んだ時のデータ点数格納用変数

dReal P[3] = {1.34, 0, 0.905};             // 先端の位置
// dReal P[3] = {0.1,0,0.1};             // 先端の位置
// 有顔ベクトル(a,b)
dReal a[3];//?わっからーん
dReal b[3] = {0.0, 0.0, 1.0};//?わっからーん
dReal T[2] = {M_PI, 0.0};
dReal THETA[NUM] = {0.0, 0.0, 0.0, 0.0, -1.1, 0.0, 0.0, 0.0, 0.0, 0.0};  // 関節の目標角度[rad]
dReal tmpTHETA3, tmpTHETA5;
dReal l[NUM] = {0.10, 0.10, 0.32, 0.435, 0.435, 0.235, 0.51, 0.51, 0.1, 0.1};   // リンクの長さ[m]

vector< POINT > pathdata;             // 経路データのxyz座標


/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = {    3.0f, 1.3f, 0.8f};          // 視点[m]
  float hpr[3] = { -160.0f, 4.5f, 0.0f};          // 視線[°]
  dsSetViewpoint(xyz, hpr);                       // 視点と視線の設定
}


int cnt = 0;
/*** シミュレーションループ ***/
void simLoop(int pause)
{
  // THETA[3] = M_PI/10;
  // P[1] = 0.5*cos(0.01*i);
  // P[2] = 1.5+0.5*sin(0.01*i);
  // P[0] = 1.0;

  #ifdef PLOT
  if (!pause) {
        PlotData d = { times, P[0], P[1], P[2] }; // x, y, z
        plotData.push_back( d );

        if (cnt%10 == 0) {
          *gnuplot << "splot '-' ls 1" << ps::endl;
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
  printSensorPosition();
  Pcontrol();                                  // P制御
  dWorldStep(world, 0.01);                     // 動力学計算
  drawArm();                                   // ロボットの描画
  drawP5();                                     // 目標位置の描画
  drawP();                                     // 目標位置の描画
  drawSensor();                                // 先端位置の描画

  printPosition(pathdata, i, 400);
  i++;
}

void command1(int cmd)
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
    case 'b':  THETA[4] += 0.1; cout << THETA[4] << endl; break;   // xキーを押すと有顔ベクトルのφが増加
    case 'n':  THETA[4] -= 0.1; cout << THETA[4] << endl; break;   // cキーを押すと有顔ベクトルのφが減少
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

  Input_Data("data/heart.dat");
#ifdef PLOT
  gnuplot = new ps::pipestream( "gnuplot -geometry 640x480 " );
  *gnuplot << "set ticslevel 0"<<ps::endl;
  *gnuplot << "set view 70, 110, 1, 1.5"<<ps::endl;
  *gnuplot << "set xrange[0.8:1.2]"<<ps::endl;
  *gnuplot << "set yrange[-0.4:0.4]"<<ps::endl;
  *gnuplot << "set zrange[0.4:1.6]"<<ps::endl;
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
  dCloseODE(); //ＯＤＥの終了
  return 0;
}
