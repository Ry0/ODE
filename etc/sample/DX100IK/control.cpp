#include "area_struct.h"
#include "control.h"

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

extern dReal P[3];             // 先端の位置
// 有顔ベクトル(a,b)
extern dReal a[3];
extern dReal b[3];
extern dReal T[2];
extern dReal THETA[NUM];  // 関節の目標角度[rad]
extern dReal tmpTHETA3, tmpTHETA5;
extern dReal l[NUM];   // リンクの長さ[m]

extern vector< POINT > pathdata;


/*** 制御 ***/
void Pcontrol()
{
  dReal k =  50.0, fMax = 5000.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    if (z >=   M_PI) z -= 2.0 * M_PI;
    if (z <= - M_PI) z += 2.0 * M_PI;
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}


// 逆運動学
void  inverseKinematics()
{
  double Px, Py, Pz;
  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
  double a2[3];
  double b2[3];
  double l2;
  double VirtualTHETA;

  l2 = 2 * l[3] * sin((M_PI - THETA[4])/2);
  // cout << "l2 =" << l2 << endl;
  VirtualTHETA = THETA[4]/2;
  // std::cout << l2 << std::endl;
  double P5x = Px - (l[8] + l[9])*a[0];
  double P5y = Py - (l[8] + l[9])*a[1];
  double P5z = Pz - (l[8] + l[9])*a[2];


  printf("Target  Position: x=%6.3f y=%6.3f z=%6.3f \n", Px, Py, Pz);

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


  switch (ANSWER) { // ANSWERはキーボードからの入力で変更
    case 1:
    case 2:
    THETA[1] = atan2(P5y, P5x);
    tmpTHETA3 = -phi - alpha;
    THETA[3] = tmpTHETA3 - VirtualTHETA;
    tmpTHETA5 = M_PI - beta - gamma;
    THETA[5] = tmpTHETA5 - VirtualTHETA; break;
    case 3:
    case 4:
    THETA[1] = atan2(P5y, P5x);
    THETA[2] = M_PI/2 - phi + alpha;
    THETA[4] = M_PI + beta; break;
    case 5:
    case 6:
    THETA[1] = atan2(P5y, P5x) + M_PI;
    THETA[2] = -(M_PI/2 - phi - alpha);
    THETA[4] = M_PI + beta; break;
    case 7:
    case 8:
    THETA[1] = atan2(P5y, P5x) + M_PI;
    THETA[2] = -(M_PI/2 - phi + alpha);
    THETA[4] = M_PI - beta; break;
  }

  a2[0] = -cos(tmpTHETA3+tmpTHETA5)*(a[0]*cos(THETA[1])+a[1]*sin(THETA[1])) + a[2]*sin(tmpTHETA3+tmpTHETA5);
  a2[1] = -a[0]*sin(THETA[1]) + a[1]*cos(THETA[1]);
  a2[2] = -sin(tmpTHETA3+tmpTHETA5)*(a[0]*cos(THETA[1])+a[1]*sin(THETA[1])) - a[2]*cos(tmpTHETA3+tmpTHETA5);
  b2[0] = -cos(tmpTHETA3+tmpTHETA5)*(b[0]*cos(THETA[1])+b[1]*sin(THETA[1])) + b[2]*sin(tmpTHETA3+tmpTHETA5);
  b2[1] = -b[0]*sin(THETA[1]) + b[1]*cos(THETA[1]);
  b2[2] = -sin(tmpTHETA3+tmpTHETA5)*(b[0]*cos(THETA[1])+b[1]*sin(THETA[1])) - b[2]*cos(tmpTHETA3+tmpTHETA5);

  switch (ANSWER) { // ANSWERはキーボードからの入力で変更
    case 1:
    case 3:
    case 5:
    case 7:
    THETA[7] = atan2(a2[1], a2[0]);
    break;
    case 2:
    case 4:
    case 6:
    case 8:
    THETA[7] = atan2(a2[1], a2[0]) + M_PI;
    break;
  }

  THETA[8] = atan2(cos(THETA[7]) * a2[0] + sin(THETA[7]) * a2[1], a2[2]);
  THETA[9] = atan2(sin(THETA[7]) * sin(THETA[7])*b2[0] - cos(THETA[7])*sin(THETA[7])*b2[1], b2[2]);
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
