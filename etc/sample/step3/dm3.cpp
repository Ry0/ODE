#include "dm3.h"

dsFunctions fn;

void start()                  /*** 前処理　***/
{
    static float xyz[3] = {3.0,0.0,1.0};         // 視点の位置
    static float hpr[3] = {-180, 0, 0};          // 視線の方向
    dsSetViewpoint(xyz,hpr);                     // カメラの設定
    dsSetSphereQuality(3);                       // 球を美しく描画
    dRSetIdentity(R);                            // 回転行列に単位行列を設定
}

/*** 描画関数の設定 ***/
void setDrawStuff(void (*function)(int pause), void (*function2)(int cmd))
{
    fn.version = DS_VERSION;    // ドロースタッフのバージョン
    fn.start   = &start;        // 前処理 start関数のポインタ
    fn.step    = function;      // simLoop関数のポインタ
    fn.command = function2;
    fn.path_to_textures = "./textures"; // テクスチャ
}

void dmLoop(int w, int h, void (*function)(int pause), void (*function2)(int cmd))
{
    setDrawStuff(function,function2);

    dsSimulationLoop(0,0,w,h,&fn);
}

