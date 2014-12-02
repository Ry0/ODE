#include "area_struct.h"
#include "control.h"

#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

extern dReal  THETA[7];                // 関節の目標角度[rad]
extern dJointID joint[7];              // ジョイントのID番号
extern int ANSWER;                     // 逆運動学の解
extern int data_num;

extern dReal P[3];                     // 先端の位置

// 有顔ベクトル(a,b)
extern dReal a[3];
extern dReal b[3];
extern dReal T[2];
extern dReal l[7];                     // リンクの長さ[m]

extern vector< POINT > vobstacle;

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
  double a2[3];
  double b2[3];
  double NowJoint[7];

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

  for (int i = 1; i < 7; ++i){
    NowJoint[i] = dJointGetHingeAngle(joint[i]);
  }

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
void printPosition(std::vector<POINT> &path, int loop)
{
  dMatrix3 tmpR;
  double R = 0.0;
  double G = 0.0;
  double B = 0.0;
  int i;
  int color = 0;
  IplImage* img = cvCreateImage( cvSize(1, 1), IPL_DEPTH_8U, 3);


  if(loop - 150 < 0){
    i = 0;
  }else{
    i = loop - 150;
  }
  for (; i < loop; ++i) {
    if((i/data_num)%2 == 0){
      P[0] = vobstacle[i%data_num].x;
      P[1] = vobstacle[i%data_num].y;
      P[2] = vobstacle[i%data_num].z;
    }else{
      P[0] = vobstacle[data_num-1-i%data_num].x;
      P[1] = vobstacle[data_num-1-i%data_num].y;
      P[2] = vobstacle[data_num-1-i%data_num].z;
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