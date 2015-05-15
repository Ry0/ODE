#include <iostream>
#include <fstream>
#include <ostream>

#include <iterator>
#include <vector>
#include <string>
#include <algorithm>

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <stdio.h>
#include <stdlib.h>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

dReal THETA[7] = {0.0};               // 関節の目標角度[rad]
dJointID joint[7];                // ジョイントのID番号

int  ANSWER = 1;                       // 逆運動学の解
int  i,j = 0;

dReal P[3] = {0.35, 0.25, 1.50};       // 先端の位置

// 有顔ベクトル(a,b)
dReal a[3];                            //?わっからーん
dReal b[3] = {0.0, 0.0, 1.0};          //?わっからーん
dReal T[2] = {M_PI, 0.0};
dReal l[7] = {0.20, 0.145, 0.33, 0.34, 0.34, 0.073, 0.18+0.04};   // リンクの長さ[m]

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

  dReal min_angle[6] = {-170.0*M_PI/180.0, -80.0*M_PI/180.0, -90.0*M_PI/180.0, -190.0*M_PI/180.0, -120.0*M_PI/180.0, -360.0*M_PI/180.0};
  dReal max_angle[6] = {170.0*M_PI/180.0, 135.0*M_PI/180.0, 155.0*M_PI/180.0, 190.0*M_PI/180.0, 120.0*M_PI/180.0, 360.0*M_PI/180.0};

  for (int i = 0; i < 6; ++i){
    if(min_angle[i] <= CalTheta[i+1] && CalTheta[i+1] <= max_angle[i]){
      CheckAnswer += 1;
    }
  }
  if(CheckAnswer == 6){
    for (int i = 1; i < 7; ++i){
      THETA[i] = CalTheta[i-1];
    }
  }
}


void PrintAngle(){
  printf("\nInput  Angle   : 1=%7.2f 2=%7.2f 3=%7.2f \n",THETA[1]*180/M_PI, THETA[2]*180/M_PI, THETA[3]*180/M_PI);
  printf("                 4=%7.2f 5=%7.2f 6=%7.2f [deg]\n\n", THETA[4] * 180 / M_PI, THETA[5] * 180 / M_PI, THETA[6] * 180 / M_PI);
}


int main(){
  inverseKinematics();
  PrintAngle();
}