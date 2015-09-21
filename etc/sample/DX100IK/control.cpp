#include "area_struct.h"
#include "control.h"
#include "PSO.h"

#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
extern int Edata_num;                      // 経路データを読み込んだ時のデータ点数格納用変数

extern dReal P[3];             // 先端の位置
// 有顔ベクトル(a,b)
extern dReal a[3];
extern dReal b[3];
extern dReal T[2];

extern dReal Smooth[idou];

extern dReal THETA[NUM];  // 関節の目標角度[rad]
extern dReal CalTheta[7];
extern dReal MinMaxTheta[7];
extern dReal tmpTHETA_L, tmpTHETA_U;
extern dReal min_theta[7]; // 各関節の最小角度[rad]
extern dReal max_theta[7];     // 各関節の最小角度[rad]
extern dReal max_thetaE, min_thetaE;

extern dReal l[NUM];   // リンクの長さ[m]

extern vector< POINT > pathdata;
extern vector< POINT > Epathdata;

// extern Particle Par;

/*** 制御 ***/
void Pcontrol()
{
  dReal k =  10.0, fMax = 10000000.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    if (z >=   M_PI) z -= 2.0 * M_PI;
    if (z <= - M_PI) z += 2.0 * M_PI;
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}


void directKinematics()
{
  int x = 0, y = 1, z = 2;
  // double l[4] = { 0.10, 0.90, 1.00, 1.00};      // リンクの長さ[m]
  double angle[8];                              // 関節の角度[rad]

  angle[1] = -dJointGetHingeAngle(joint[1]);     // 第1関節角度の取得
  angle[2] = -dJointGetHingeAngle(joint[3]);     // 第2関節角度の取得
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


// 逆運動学
bool inverseKinematics(double Theta[])
{
  double Px, Py, Pz;
  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
  double a2[3];
  double b2[3];

  double l2;
  double VirtualTHETA;

  l2 = 2 * l[3] * sin((M_PI - Theta[2])/2);
  // cout << "l2 =" << l2 << endl;
  VirtualTHETA = Theta[2]/2;
  // std::cout << l2 << std::endl;
  double P5x = Px - (l[8] + l[9])*a[0];
  double P5y = Py - (l[8] + l[9])*a[1];
  double P5z = Pz - (l[8] + l[9])*a[2];


  // printf("Target  Position: x=%6.3f y=%6.3f z=%6.3f \n", Px, Py, Pz);

  double tmpL  = sqrt(P5x * P5x + P5y * P5y);
  double P1P   = sqrt((P5z - l[0] - l[1])*(P5z - l[0] - l[1]) + pow((tmpL - l[2]),2));
  // cout << "P1P = " << P1P << endl;
  double Ca    = (l2 * l2 + P1P * P1P - l[5] * l[5] - (l[6]+l[7]) * (l[6]+l[7]) )/(2 * l2 * P1P);  // cosα
  // cout << "Ca = " << Ca << endl;
  double phi   = atan2(P5z - (l[0]+l[1]), tmpL - l[2]);                      //φ
  // cout << "Phi = " << phi << endl;
  double alpha = atan2(sqrt(1 - Ca * Ca), Ca);                         //α
  // cout << "alpha = " << alpha << endl;

  double Cb    = (l2 * l2 + l[5] * l[5] + (l[6]+l[7])*(l[6]+l[7]) - P1P * P1P)/(2 * l2 * sqrt(l[5]*l[5] + (l[6]+l[7])*(l[6]+l[7])));  //cosβ
  // cout << "Cb = " << Cb << endl;
  double beta  = atan2(sqrt(1- Cb * Cb), Cb);                          //β
  // cout << "beta =" << beta << endl;
  double gamma = atan2(l[6]+l[7], l[5]);                                    //γ


  Theta[0] = atan2(P5y, P5x);
  tmpTHETA_L = - phi - alpha;
  Theta[1] = tmpTHETA_L - VirtualTHETA;
  tmpTHETA_U = M_PI - beta - gamma;
  Theta[3] = tmpTHETA_U - VirtualTHETA;

  a2[0] = -cos(tmpTHETA_L+tmpTHETA_U)*(a[0]*cos(Theta[0])+a[1]*sin(Theta[0])) + a[2]*sin(tmpTHETA_L+tmpTHETA_U);
  a2[1] = -a[0]*sin(Theta[0]) + a[1]*cos(Theta[0]);
  a2[2] = -sin(tmpTHETA_L + tmpTHETA_U) * (a[0] * cos(Theta[0]) + a[1] * sin(Theta[0])) -
          a[2] * cos(tmpTHETA_L + tmpTHETA_U);
  b2[0] = -cos(tmpTHETA_L + tmpTHETA_U) * (b[0] * cos(Theta[0]) + b[1] * sin(Theta[0])) +
          b[2] * sin(tmpTHETA_L + tmpTHETA_U);
  b2[1] = -b[0] * sin(Theta[0]) + b[1] * cos(Theta[0]);
  b2[2] = -sin(tmpTHETA_L + tmpTHETA_U) * (b[0] * cos(Theta[0]) + b[1] * sin(Theta[0])) -
          b[2] * cos(tmpTHETA_L + tmpTHETA_U);

  Theta[4] = atan2(a2[1], a2[0]);

  Theta[5] = atan2(cos(Theta[4]) * a2[0] + sin(Theta[4]) * a2[1], a2[2]);
  Theta[6] = atan2(sin(Theta[4]) * sin(Theta[4]) * b2[0] - cos(Theta[4]) * sin(Theta[4]) * b2[1],b2[2]);
  if(CheckThetaE()){
    return true;
  }else{
    return false;
  }
}

bool CheckThetaE()
{
  int CheckAnswer = 0;

  for (int i = 0; i < 7; ++i) {
    if (min_theta[i] <= CalTheta[i] && CalTheta[i] <= max_theta[i]) {
      CheckAnswer += 1;
    } else {
      // cout << i << "番目範囲外" << endl;
    }
  }

  if (CheckAnswer == 7) {
    // THETA[1] = CalTheta[0];
    // THETA[3] = CalTheta[1];
    // THETA[4] = CalTheta[2];
    // THETA[5] = CalTheta[3];
    // THETA[7] = CalTheta[4];
    // THETA[8] = CalTheta[5];
    // THETA[9] = CalTheta[6];
    return true;
  } else {
    // cout << "動作範囲外" << endl;
    return false;
  }
  // cout << "THETA_S = " << THETA[1] * 180 / M_PI << endl;
  // cout << "THETA_L = " << THETA[3] * 180 / (M_PI) << endl;
  // cout << "THETA_E = " << THETA[4] * 180 / (M_PI) << endl;
  // cout << "THETA_U = " << THETA[5] * 180 / (M_PI) << endl;
  // cout << "THETA_R = " << THETA[7] * 180/(M_PI) << endl;
  // cout << "THETA_B = " << THETA[8] * 180/(M_PI) << endl;
  // cout << "THETA_T = " << THETA[9] * 180/(M_PI) << endl;
  // cout << endl;
  // cout << "THETA_S = " << CalTheta[0]*180/M_PI << endl;
  // cout << "THETA_L = " << CalTheta[1]*180/M_PI << endl;
  // cout << "THETA_E = " << CalTheta[2]*180/M_PI << endl;
  // cout << "THETA_U = " << CalTheta[3]*180/M_PI << endl;
  // cout << "THETA_R = " << CalTheta[4]*180/M_PI << endl;
  // cout << "THETA_B = " << CalTheta[5]*180/M_PI << endl;
  // cout << "THETA_T = " << CalTheta[6]*180/M_PI << endl;
  // cout << endl;
}


void CheckTheta(){
  int CheckAnswer = 0;

  for(double j = -45; j <= 120; j+=0.1){
    CheckAnswer = 0;
    MinMaxTheta[2] = j*M_PI/180.0;
    inverseKinematics(MinMaxTheta);
    for (int i = 0; i < 7; ++i){
      if(min_theta[i] <= MinMaxTheta[i] && MinMaxTheta[i] <= max_theta[i]){
        CheckAnswer += 1;
      }
      // }else{
      //   cout << i << "番目範囲外: " << CalTheta[2]*180/(M_PI) << endl;
      // }
    }

    if(CheckAnswer==7){
      // cout << "OK: " << CalTheta[2] * 180 / (M_PI) << endl;
      min_thetaE = MinMaxTheta[2];
      break;
    }
  }

  for(double j = 120; j >= -45; j-=0.1){
    CheckAnswer = 0;
    MinMaxTheta[2] = (double)j*M_PI/180.0;
    inverseKinematics(MinMaxTheta);
    for (int i = 0; i < 7; ++i){
      if(min_theta[i] <= MinMaxTheta[i] && MinMaxTheta[i] <= max_theta[i]){
        CheckAnswer += 1;
      }
      // }else{
      //   cout << i << "番目範囲外: " << CalTheta[2]*180/(M_PI) << endl;
      // }
    }

    if(CheckAnswer==7){
      // cout << "OK: " << CalTheta[2]*180/(M_PI) << endl;
      max_thetaE = MinMaxTheta[2];
      break;
    }
  }
}


double AdjustTheta(){
  int CheckAnswer = 0;
  bool success_flag = false;
  double adjust;
  dReal Theta[7]={0.0};

  for(double j = -45; j <= 120; j+=0.1){
    CheckAnswer = 0;
    Theta[2] = j*M_PI/180.0;
    inverseKinematics(Theta);
    for (int i = 0; i < 7; ++i){
      if(min_theta[i] <= Theta[i] && Theta[i] <= max_theta[i]){
        CheckAnswer += 1;
      }
      // }else{
      //   cout << i << "番目範囲外: " << CalTheta[2]*180/(M_PI) << endl;
      // }
    }

    if(CheckAnswer==7){
      // cout << "OK: " << CalTheta[2] * 180 / (M_PI) << endl;
      adjust = Theta[2];
      success_flag = true;
      break;
    }
  }

  if(success_flag){
    return adjust;
  }else{
    return -100000;
  }


}


void OptimizationThetaE(int i){
  double sum = 0.0;
  Particle Par;
  double tmpindex[idou];

  Par = ExecPSO(min_thetaE*180/(M_PI), max_thetaE*180/(M_PI));

  // cout << "Eの角度 = " << Par->x_star[0] << endl;
  // cout << "Eの角度 = " << floor(tmp)/100.0*(M_PI)/180 << endl;
  CalTheta[2] = Par->x_star[0]*(M_PI)/180;
  // if(i<idou){
  //   CalTheta[2] = Par->x_star[0]*(M_PI)/180;
  //   Smooth[i%idou] = CalTheta[2];
  // }else{
  //   Smooth[i%idou] = Par->x_star[0]*(M_PI)/180;

  //   for (int j = 0; j < idou; ++j){
  //     tmpindex[j] = Smooth[j];
  //   }
  //   sort(tmpindex, tmpindex+idou);
  //   sum = 0.0;
  //   for (int k = 0+2; k < idou-2; ++k)
  //   {
  //     sum += tmpindex[k];
  //   }

  //   CalTheta[2] = sum/(idou-2-2);
  // }

}

void yugan_a()
{
  a[0] = sin(T[0])*cos(T[1]);
  a[1] = sin(T[0])*sin(T[1]);
  a[2] = cos(T[0]);
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
  cout << "ファイル読み込み成功" << endl;

  input.close();
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
    dsDrawSphere(P, tmpR, 0.02);
  }
}


void EInput_Data(std::string fileName)
{
  std::ifstream input(fileName.c_str());
  Edata_num = CountNumbersOfTextLines(fileName);
  Epathdata.resize(Edata_num);

  for (int i = 0; i < Edata_num; ++i){
    input >> Epathdata[i].x;
  }
  cout << "ファイル読み込み成功" << endl;

  input.close();
}
