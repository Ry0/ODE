#include "area_struct.h"
#include "control.h"
#include "obstacle_struct.h"

#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/*---------------------------------------------------------------------- ↓グローバル変数定義ここから↓ -------------------------------------------------------------------------------*/
extern dWorldID world;                   // 動力学計算用のワールド

extern dReal  THETA[7];                // 関節の目標角度[rad]
extern dReal min_angle[6]; // 各関節の最小角度[rad]
extern dReal max_angle[6]; // 各関節の最小角度[rad]
extern dJointID joint[7];              // ジョイントのID番号
extern int ANSWER;                     // 逆運動学の解
extern int data_num;

extern dReal P[3];                     // 先端の位置

// 有顔ベクトル(a,b)
extern dReal a[3];
extern dReal b[3];
extern dReal T[2];
extern dReal l[7];                     // リンクの長さ[m]

extern vector< POINT > pathdata;
extern dReal StartP[3],GoalP[3];


extern vector< POINT > pStart;
extern vector< POINT > pEnd;
extern vector< POINT > RRT_path;
extern vector< POINT > RRT_path_mod;
extern int RRT_node_num;
extern int RRT_path_num;
extern int RRT_path_mod_num;
/*---------------------------------------------------------------------- ↑グローバル変数定義ここまで↑ -------------------------------------------------------------------------------*/


bool YesorNo(){
  string Confirm;

  cout << "上のモードで始めます。よければ「y」、だめなら「n」を入力してEnter" << endl;
  while(1){
    cin >> Confirm;
    if(Confirm == "n"){
      return false;
      break;
    }else if(Confirm == "y"){
      return true;
      break;
    }else{
      cout << "「y」か「n」を入力してください" << endl;
    }
  }
}


int input_arg(int argc, char* argv[])
{
  string PlotDataPass = "./data/";
  string filename;

  if (argc <= 1) {
    cout << "\n自由操作モード" << endl;
    return 0;
  } else if (argc == 2) { // 引数を一つ指定されたら、そのファイルにしたがって軌道生成+障害物を立てる
    string ObstacleFile = string(argv[1]);
    filename = PlotDataPass + ObstacleFile;
    cout << "ファイルパスは " << filename << endl;
    Input_Data(filename);
    cout << "\n障害物回避モード" << endl;
    return 1;
  } else {
    cout << "引数は1つだけにしてくださいな。" << endl;
    return 2;
  }
}


int CountNumbersOfTextLines(std::string fileName){
  int i = 0;
  std::ifstream ifs(fileName);         // ファイルを開く。

  if(ifs){                             // ファイルのオープンに成功していれば、これは file は true を返す。
    std::string line;                  // 1行ごとの文字列を格納するための string オブジェクト。
    while(true){
      getline( ifs, line );            // 1行読み取る。
      if( ifs.eof() ){                 // ファイルが終端まで来たら、break 文を実行して while 文を抜ける。
        break;
      }
      i++;                             // 1行読み取ったので、インクリメントする。
    }
  }

  return i;                            // カウントした行数を返す。
}



void Input_Data(std::string fileName)
{
  std::ifstream input(fileName.c_str());
  data_num = CountNumbersOfTextLines(fileName);
  pathdata.resize(data_num);

  for (int i = 0; i < data_num; ++i){
    input >> pathdata[i].x >> pathdata[i].y >> pathdata[i].z;
  }

  StartP[0] = pathdata[0].x;
  StartP[1] = pathdata[0].y;
  StartP[2] = pathdata[0].z;
  GoalP[0] = pathdata[data_num-1].x;
  GoalP[1] = pathdata[data_num-1].y;
  GoalP[2] = pathdata[data_num-1].z;
  cout << "ファイル読み込み成功" << endl;

  input.close();
}



void Input_RRT_Data(std::string fileName)
{
  std::ifstream input(fileName.c_str());
  RRT_node_num = CountNumbersOfTextLines(fileName);

  pStart.resize(RRT_node_num/4);
  pEnd.resize(RRT_node_num/4);

  cout << "行数 = " << RRT_node_num << endl;
  for (int i = 0; i < RRT_node_num/4; ++i){
    input >> pStart[i].x >> pStart[i].y >> pStart[i].z;
    input >> pEnd[i].x >> pEnd[i].y >> pEnd[i].z;
  }

  input.close();
}



void printRRT()
{
  double R = 169.0/255.0;
  double G = 169.0/255.0;
  double B = 169.0/255.0;

  double p1[3], p2[3];

  for (int i = 0; i < RRT_node_num/4; ++i){
    p1[0] = pStart[i].x;
    p1[1] = pStart[i].y;
    p1[2] = pStart[i].z;
    p2[0] = pEnd[i].x;
    p2[1] = pEnd[i].y;
    p2[2] = pEnd[i].z;
    dsSetColorAlpha(R, G, B, 0.3);
    dsDrawLineD(p1, p2);
  }
}



void Input_RRTPath_Data(std::string fileName)
{
  std::ifstream input(fileName.c_str());
  RRT_path_num = CountNumbersOfTextLines(fileName);
  RRT_path.resize(RRT_path_num);

  for (int i = 0; i < RRT_path_num; ++i){
    input >> RRT_path[i].x >> RRT_path[i].y >> RRT_path[i].z;
  }

  cout << "RRT_path_num = " << RRT_path_num << endl;
  input.close();
}



void printPath()
{
  double R = 255.0/255.0;
  double G = 51.0/255.0;
  double B = 0.0/255.0;

  double p1[3], p2[3];

  for (int i = 1; i < RRT_path_num; ++i){
    p1[0] = RRT_path[i-1].x;
    p1[1] = RRT_path[i-1].y;
    p1[2] = RRT_path[i-1].z;
    p2[0] = RRT_path[i].x;
    p2[1] = RRT_path[i].y;
    p2[2] = RRT_path[i].z;
    dsSetColorAlpha(R, G, B, 0.3);
    dsDrawLineD(p1, p2);
  }
}



void Input_RRTPath_mod_Data(std::string fileName)
{
  std::ifstream input(fileName.c_str());
  RRT_path_mod_num = CountNumbersOfTextLines(fileName);
  RRT_path_mod.resize(RRT_path_num);

  for (int i = 0; i < RRT_path_num; ++i){
    input >> RRT_path_mod[i].x >> RRT_path_mod[i].y >> RRT_path_mod[i].z;
  }

  cout << "RRT_path_mod_num = " << RRT_path_mod_num << endl;
  input.close();
}



void printPath_mod()
{
  double R = 0./255.0;
  double G = 0.0/255.0;
  double B = 204.0/255.0;

  double p1[3], p2[3];

  for (int i = 1; i < RRT_path_mod_num; ++i){
    p1[0] = RRT_path_mod[i-1].x;
    p1[1] = RRT_path_mod[i-1].y;
    p1[2] = RRT_path_mod[i-1].z;
    p2[0] = RRT_path_mod[i].x;
    p2[1] = RRT_path_mod[i].y;
    p2[2] = RRT_path_mod[i].z;
    dsSetColorAlpha(R, G, B, 0.3);

    dsDrawLineD(p1, p2);
  }
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


void Vcontrol()
{
  //dReal k =  20.0;
  dReal fMax = 200.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < 7; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    if (z >=   M_PI) z -= 2.0 * M_PI;
    if (z <= - M_PI) z += 2.0 * M_PI;
    if(z > 0.01){
      dJointSetHingeParam(joint[j],dParamVel, 1);  // 角速度の設定
      dJointSetHingeParam(joint[j],dParamFMax, fMax); // トルクの設定
    }else if(z < -0.01){
      dJointSetHingeParam(joint[j],dParamVel, -1);  // 角速度の設定
      dJointSetHingeParam(joint[j],dParamFMax, fMax); // トルクの設定)else{
    }else{
      dJointSetHingeParam(joint[j],dParamVel, 0);  // 角速度の設定
      dJointSetHingeParam(joint[j],dParamFMax, fMax); // トルクの設定
    }
  }
}



void inverseKinematics()
{
  double Px, Py, Pz;
  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
  dReal CalTheta[7] = {0.0};       // 目標角度計算用
  int CheckAnswer = 0;

  double a2[3];
  double b2[3];

  double P5x = Px - (l[5] + l[6])*a[0];
  double P5y = Py - (l[5] + l[6])*a[1];
  double P5z = Pz - (l[5] + l[6])*a[2];

  // printf("Target  Position: x=%7.3f y=%7.3f z=%7.3f \n", Px, Py, Pz);

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
      CalTheta[1] = atan2(P5y, P5x);
      CalTheta[2] = M_PI/2 - phi - alpha;
      CalTheta[3] = M_PI - beta; break;
    case 3:
    case 4:
      CalTheta[1] = atan2(P5y, P5x);
      CalTheta[2] = M_PI/2 - phi + alpha;
      CalTheta[3] = M_PI + beta; break;
    case 5:
    case 6:
      CalTheta[1] = atan2(P5y, P5x) + M_PI;
      CalTheta[2] = -(M_PI/2 - phi - alpha);
      CalTheta[3] = M_PI + beta; break;
    case 7:
    case 8:
      CalTheta[1] = atan2(P5y, P5x) + M_PI;
      CalTheta[2] = -(M_PI/2 - phi + alpha);
      CalTheta[3] = M_PI - beta; break;
  }

  a2[0] = cos(CalTheta[2]+CalTheta[3])*(a[0]*cos(CalTheta[1])+a[1]*sin(CalTheta[1])) - a[2]*sin(CalTheta[2]+CalTheta[3]);
  a2[1] = -a[0]*sin(CalTheta[1]) + a[1]*cos(CalTheta[1]);
  a2[2] = sin(CalTheta[2]+CalTheta[3])*(a[0]*cos(CalTheta[1])+a[1]*sin(CalTheta[1])) + a[2]*cos(CalTheta[2]+CalTheta[3]);
  b2[0] = cos(CalTheta[2]+CalTheta[3])*(b[0]*cos(CalTheta[1])+b[1]*sin(CalTheta[1])) - b[2]*sin(CalTheta[2]+CalTheta[3]);
  b2[1] = -b[0]*sin(CalTheta[1]) + b[1]*cos(CalTheta[1]);
  b2[2] = sin(CalTheta[2]+CalTheta[3])*(b[0]*cos(CalTheta[1])+b[1]*sin(CalTheta[1])) + b[2]*cos(CalTheta[2]+CalTheta[3]);

  switch (ANSWER) { // ANSWERはキーボードからの入力で変更
    case 1:
    case 3:
    case 5:
    case 7:
      CalTheta[4] = atan2(a2[1], a2[0]);
      break;
    case 2:
    case 4:
    case 6:
    case 8:
      CalTheta[4] = atan2(a2[1], a2[0]) + M_PI;
      break;
  }

  CalTheta[5] = atan2(cos(CalTheta[4]) * a2[0] + sin(CalTheta[4]) * a2[1], a2[2]);
  CalTheta[6] = atan2(sin(CalTheta[4]) * sin(CalTheta[5])*b2[0] - cos(CalTheta[4])*sin(CalTheta[5])*b2[1], b2[2]);

  for (int i = 0; i < 6; ++i){
    if(min_angle[i] <= CalTheta[i+1] && CalTheta[i+1] <= max_angle[i]){
      CheckAnswer += 1;
    }
  }

  if(CheckAnswer == 6){
    for (int i = 1; i < 7; ++i){
      THETA[i] = CalTheta[i];
    }
  }

  #ifdef PrintStatus
  dReal  NowJoint[7] = {0.0};
  for (int i = 1; i < 7; ++i){
    NowJoint[i] = dJointGetHingeAngle(joint[i]);
  }
  PrintAngle(NowJoint);
  #endif
}


void PrintAngle(dReal NowJoint[]){
  printf("\nInput  Angle   : 1=%7.2f 2=%7.2f 3=%7.2f \n",THETA[1]*180/M_PI,THETA[2]*180/M_PI,THETA[3]*180/M_PI);
  printf("                 4=%7.2f 5=%7.2f 6=%7.2f [deg]\n\n", THETA[4] * 180 / M_PI, THETA[5] * 180 / M_PI, THETA[6] * 180 / M_PI);
  printf("\nOutput Angle   : 1=%7.2f 2=%7.2f 3=%7.2f \n",NowJoint[1] * 180 / M_PI, NowJoint[2]*180/M_PI, NowJoint[3]*180/M_PI);
  printf("                 4=%7.2f 5=%7.2f 6=%7.2f [deg]\n\n", NowJoint[4] * 180 / M_PI, NowJoint[5] * 180 / M_PI, NowJoint[6] * 180 / M_PI);
}



void yugan_a()
{
  a[0] = sin(T[0]) * cos(T[1]);
  a[1] = sin(T[0]) * sin(T[1]);
  a[2] = cos(T[0]);
}



#define B(IMG, X, Y) ((uchar*)((IMG)->imageData + (IMG)->widthStep*(Y)))[(X)*3]
#define G(IMG, X, Y) ((uchar*)((IMG)->imageData + (IMG)->widthStep*(Y)))[(X)*3+1]
#define R(IMG, X, Y) ((uchar*)((IMG)->imageData + (IMG)->widthStep*(Y)))[(X)*3+2]
void printPosition(std::vector<POINT> &path, int loop, int DrawLength)
{
  dMatrix3 tmpR;
  double R = 0.0;
  double G = 0.0;
  double B = 0.0;
  int i;
  int color = 0;
  IplImage* img = cvCreateImage( cvSize(1, 1), IPL_DEPTH_8U, 3);


  if(loop - DrawLength < 0){
    i = 0;
  }else{
    i = loop - DrawLength;
  }
  for (; i < loop; ++i) {
    if((i/data_num)%2 == 0){
      P[0] = pathdata[i%data_num].x;
      P[1] = pathdata[i%data_num].y;
      P[2] = pathdata[i%data_num].z;
    }else{
      P[0] = pathdata[data_num-1-i%data_num].x;
      P[1] = pathdata[data_num-1-i%data_num].y;
      P[2] = pathdata[data_num-1-i%data_num].z;
    }
    color = i%180;
    B(img, 0, 0) = color;
    G(img, 0, 0) = 255;
    R(img, 0, 0) = 255;

    cvCvtColor(img, img, CV_HSV2BGR);

    R = R(img, 0, 0) / 255.0;
    G = G(img, 0, 0) / 255.0;
    B = B(img, 0, 0) / 255.0;
    dsSetColor(R, G, B);
    dRSetIdentity(tmpR);
    dsDrawSphere(P, tmpR, 0.008);
  }
}
