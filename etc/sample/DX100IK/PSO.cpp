#include "PSO.h"
#include "area_struct.h"
#include "arm_struct.h"
#include "control.h"

extern dReal P[3];             // 先端の位置
// 有顔ベクトル(a,b)
extern dReal a[3];
extern dReal b[3];
extern dReal T[2];
// extern double l[10];   // リンクの長さ[m]
extern dReal CalTheta[7];    // 目標角度計算用
extern dReal MinMaxTheta[7];
// extern double MinMaxTheta[7]; // 関節角度の最小値，最大値計算用
// extern double tmpTHETA_L, tmpTHETA_U;
// extern Particle Par;


double GetRandom(double min,double max, int digit){
  double R,ten;

  ten = pow(10, digit-1);
  R = min*ten + (int)(rand()*((max-min)*ten+1.0)/(1.0+RAND_MAX));
  return R/ten;
}

/* 評価値計算 */
void Evaluate(Particle P){

  // double weight[4] = {4.35, 4.35, 2.35 + 10.2, 2.0};  // 質量
  // double length[4] = {4.35, 4.35, sqrt(2.35 * 2.35 + 10.2 *10.2) , 2.0};  // 質量

  /* 各座標原点から見た重心位置ベクトル(XY平面上) */
  double gL_x = 0.217;  double gL_y = -0.024;
  double gE_x = 0.088;  double gE_y =  0.007;
  double gU_x = 0.141;  double gU_y = -0.118;
  double gR_x = 0.0;    double gR_y =  0.026;
  double gB_x = 0.0;    double gB_y = -0.104;
  // double gT_x = 0.0;    double gT_y =  0.0;

  /* 各座標原点から見た重心位置ベクトルまでの長さ */
  double leng_Lg = pow((gL_x*gL_x + gL_y*gL_y),0.5);
  double leng_Eg = pow((gE_x*gE_x + gE_y*gE_y),0.5);
  double leng_Ug = pow((gU_x*gU_x + gU_y*gU_y),0.5);
  double leng_Bg = pow((gB_x*gB_x + gB_y*gB_y),0.5);
  double leng_Rg = pow((gR_x*gR_x + gR_y*gR_y),0.5);

  /* 各リンク質量(アクチュエータを含む) */
  double m1 = 77.063  * 9.8;
  double m2 = 150.797 * 9.8;
  double m3 = 167.57  * 9.8;
  double m4 = 40.47   * 9.8;
  double m5 = 16.96   * 9.8;
  double m7 = 1.9     * 9.8;

  /* 各リンク長 */
  double len1 = 0.453;
  double len2 = 0.453;
  double a4 = 0.235;
  double d4 = 1.02;
  double len4 = 0.2;
  double len5 = 0.0;

  /*モータの定格トルク*/
  double cos_trq_L = 290; // L軸 [kgfcm]
  double cos_trq_E = 290; // E軸
  double cos_trq_U = 189; // U軸
  double cos_trq_B = 85;  // B軸

  /*減速比*/
  double redu_L = 209; // L軸 [kgfcm]
  double redu_E = 209; // E軸
  double redu_U = 171; // U軸
  double redu_B = 52.727;  // B軸

  /*定格最大出力[トルク]*/
  double trq_max_L = cos_trq_L * 0.0981 * redu_L;
  double trq_max_E = cos_trq_E * 0.0981 * redu_E;
  double trq_max_U = cos_trq_U * 0.0981 * redu_U;
  double trq_max_B = cos_trq_B * 0.0981 * redu_B;

  double Trq_L, Trq_E, Trq_U, Trq_B;

  // double CostFn = 0.0;
  double s1 = 0.0, s2 = 0.0, s3 = 0.0, s4 = 0.0, s5 = 0.0;

  dReal PSOTheta[7];

  PSOTheta[2] = P->x[0]*M_PI/180;
  // cout << "評価関数計算:" << P->x[0] << endl;
  inverseKinematics(PSOTheta);

  s1 = -PSOTheta[1]-M_PI/2.0;
  s2 = -PSOTheta[2];
  s3 = -PSOTheta[3];
  s4 = -PSOTheta[5];

  // L軸周りの負荷
  Trq_L = m1*leng_Lg *sin(-s1+(atan2(0.217,-0.024)-M_PI/2))+m2*(len1*sin(-s1)+leng_Eg * sin(-s1-s2+(atan2(0.088,0.007)-M_PI/2)))+m3*(len1*sin(-s1)+len2*sin(-s1-s2)+leng_Ug*sin(-s1-s2-s3+(atan2(0.141,-0.118)-M_PI/2)))+m4*(len1*sin(-s1)+len2*sin(-s1-s2)+a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+leng_Rg*sin(-s1-s2-s3+M_PI/2+(atan2(0,0.026)-M_PI/2)))+m5*(len1*sin(-s1)+len2*sin(-s1-s2)+a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+len4*sin(-s1-s2-s3-s4+M_PI/2)+leng_Bg*sin(-s1-s2-s3-s4+M_PI/2+(atan2(0,-0.104)-M_PI/2)))+m7*(len1*sin(-s1)+len2*sin(-s1-s2)+a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+len4*sin(-s1-s2-s3-s4+M_PI/2)+len5*sin(-s1-s2-s3-s4-s5+M_PI/2));

  // E軸周りの負荷
  Trq_E = m2*(leng_Eg * sin(-s1-s2+(atan2(0.088,0.007)-M_PI/2)))+m3*(len2*sin(-s1-s2)+leng_Ug*sin(-s1-s2-s3+(atan2(0.141,-0.118)-M_PI/2)))+m4*(len2*sin(-s1-s2)+a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+leng_Rg*sin(-s1-s2-s3+M_PI/2+(atan2(0,0.026)-M_PI/2)))+m5*(len2*sin(-s1-s2)+a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+len4*sin(-s1-s2-s3-s4+M_PI/2)+leng_Bg*sin(-s1-s2-s3-s4+M_PI/2+(atan2(0,-0.104)-M_PI/2)))+m7*(len2*sin(-s1-s2)+a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+len4*sin(-s1-s2-s3-s4+M_PI/2)+len5*sin(-s1-s2-s3-s4-s5+M_PI/2));

  // U軸周りの負荷
  Trq_U = m3*(leng_Ug*sin(-s1-s2-s3+(atan2(0.141,-0.118)-M_PI/2)))+m4*(a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+leng_Rg*sin(-s1-s2-s3+M_PI/2+(atan2(0,0.026)-M_PI/2)))+m5*(a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+len4*sin(-s1-s2-s3-s4+M_PI/2)+leng_Bg*sin(-s1-s2-s3-s4+M_PI/2+(atan2(0,-0.104)-M_PI/2)))+m7*(a4*sin(-s1-s2-s3)+d4*sin(-s1-s2-s3+M_PI/2)+len4*sin(-s1-s2-s3-s4+M_PI/2)+len5*sin(-s1-s2-s3-s4-s5+M_PI/2));

  // B軸周りの負荷
  Trq_B = m4*(leng_Rg*sin(-s1-s2-s3+M_PI/2+(atan2(0,0.026)-M_PI/2)))+m5*(len4*sin(-s1-s2-s3-s4+M_PI/2)+leng_Bg*sin(-s1-s2-s3-s4+M_PI/2+(atan2(0,-0.104)-M_PI/2)))+m7*(len4*sin(-s1-s2-s3-s4+M_PI/2)+len5*sin(-s1-s2-s3-s4-s5+M_PI/2));

  // plot << CalTheta[2] << "\t" << Torque[0] << "\t" << Torque[1] << "\t" << Torque[2] << "\t" << Torque[3] << endl;
  P->f = 0.0;
  P->f = fabs(Trq_L/trq_max_L) + fabs(Trq_E/trq_max_E) + fabs(Trq_U/trq_max_U) + fabs(Trq_B/trq_max_B);
  // cout << P->x[0] << ", " << P->f << endl;
  // P->f = -4*exp(-(pow((P->x[0]-2),2) + pow((P->x[1]-2),2))) - 2*exp(-(pow((P->x[0]+0.5),2) + pow((P->x[1]+0.5),2))/3) - 3.5*exp(-(pow((P->x[0]-4),2) + pow((P->x[1]+2),2))/2);
}

/* pbestの更新 */
void UpdateBest(Particle P){
  int j;

  for(j=0; j<Nvariables; j++){
    P->x_star[j]=P->x[j];
  }
  P->pbest=P->f;
}

/* それぞれのお魚さんの情報まとめ */
int Initialize(Particle P, int n, double min, double max){
  int i, j;
  int G;//もっともよいお魚さんの個体値番号

  G=0;
  for(i=0; i<n; i++) {
    for(j=0; j<Nvariables; j++){
      P[i].x[j] = GetRandom(min, max, 3);//0から1の乱数発生
      P[i].v[j] = 0.0;//速度は0
    }
    Evaluate(&P[i]);
    UpdateBest(&P[i]);
    if(P[i].f < P[G].f){//Gよりもi番目のお魚さんの評価値がよかったらGをiに代入
      G = i;
    }
  }
  return G;
}

/* 新しいデータ構造の割り当て */
#define New(type, n, msg) (type *)NewCell(sizeof(type), n, msg)

void *NewCell(int size, int n, char *msg){
  void *NEW;
  //newの要素をn個分確保して、それがNULLだったらerrorを返す
  if((NEW=malloc(size*n))==NULL){
    fprintf(stderr, "Cannot allocate memory for %d %s\n", n, msg);
    exit(1);
  }
  return NEW;
}

/* パーティクル型のデータ構造の新しい割り当て */
Particle NewParticles(int n){
  int i;
  Particle P;

  P = New(ParticleRec, n, (char*)"particles");
  for(i=0; i<n; i++){
    P[i].x = New(double, Nvariables, (char*)"x");
    P[i].v = New(double, Nvariables, (char*)"v");
    P[i].x_star = New(double, Nvariables, (char*)"x*");
  }
  return P;
}

/* 発生させたお魚さんの位置と最もよい場所の表示 */
void Print(Particle P){
  int j;

  for(j=0; j<Nvariables; j++){
    printf("%f ", P->x_star[j]);
  }
  printf(" = %g\n", P->pbest);
}


Particle ExecPSO(double min, double max){
  int t, i, j;
  Particle P;
  int G;
  double w;

  P = NewParticles(Nparticles);
  G = Initialize(P, Nparticles, min, max);
  w=W_0;

  for(t=1; t<=T_MAX; t++){

    for(i=0; i<Nparticles; i++){

      for(j=0; j<Nvariables; j++){

        P[i].v[j] = w*P[i].v[j]
        + C1*(double)rand()/RAND_MAX*(P[i].x_star[j]-P[i].x[j])
        + C2*(double)rand()/RAND_MAX*(P[G].x_star[j]-P[i].x[j]);
        if(P[i].v[j] < -MAX_V){
          P[i].v[j] = -MAX_V;
        }else if(P[i].v[j] > MAX_V){
          P[i].v[j] = MAX_V;
        }
        P[i].x[j] += P[i].v[j];
        // お魚さんがエリアオーバーしたら漁場に戻す
        if(P[i].x[j] <= min){
          P[i].x[j] = min;
        }
        if(P[i].x[j] >= max){
          P[i].x[j] = max;
        }

      }
      Evaluate(&P[i]);
      if(P[i].f < P[i].pbest){
        if(P[i].f < P[G].pbest) G = i;
        UpdateBest(&P[i]);
      }

    }

    w -= (W_0-W_T)/T_MAX;

  }

  return P;
}


