#ifndef PSO_H_
#define PSO_H_
#include "area_struct.h"

using namespace std;

#define TIME

/*パーティクルの構造体*/
typedef struct {
  double *x;//x_i:i番目のパーティクル(お魚さん)の位置(N次元空間の位置x)
  double *v;//v_i:i番目のお魚さんの速度
  double f;//評価値
  double pbest;//最もよい評価値
  double *x_star;//最も評価が高い位置データ
} ParticleRec, *Particle;

/* パラメータの定義 */
//パーティクルの数
#define Nparticles 10
//ループ回数
#define T_MAX 100
//慣性重み(以前の速度をどれだけ保持するのか) t=0 (W_0) t=T_MAX (W_T)
#define W_0 0.9
#define W_T 0.4
#define MAX_V 2.0
//c1は自分自身の最良の場所に重きをおいて、c2は群れの最良の場所に重みをおくか
#define C1 2.0
#define C2 2.0
//次元数
#define Nvariables 1

// int quit_flag = 0;
double GetRandom(double min,double max, int digit);

/* 評価値計算 */
void Evaluate(Particle P);

/* pbestの更新 */
void UpdateBest(Particle P);

/* それぞれのお魚さんの情報まとめ */
int Initialize(Particle P, int n, double min, double max);

/* 新しいデータ構造の割り当て */
#define New(type, n, msg) (type *)NewCell(sizeof(type), n, msg)
void *NewCell(int size, int n, char *msg);

/* パーティクル型のデータ構造の新しい割り当て */
Particle NewParticles(int n);

/* 発生させたお魚さんの位置と最もよい場所の表示 */
void Print(Particle P);

/* PSOの実行 */
Particle ExecPSO(double min, double max);

#endif