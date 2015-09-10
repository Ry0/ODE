/* step5 ドミノ倒し 　*/
#include "dm5.h"
#define DOMINO_NUM  1000

static int STEPS = 0;   // シミュレーションのステップ数
double red[3] = {1.3, 0.0, 0.0}; // 赤色
dmObject domino[DOMINO_NUM]; // ドミノ

void simLoop(int pause)           /***  シミュレーションループ　***/
{

    int i;
    double x = 5.0, y = 0, z = 1; // カメラの位置[m]
    double yaw = -180, pitch = 0, roll = 0; // カメラの姿勢　ヨー，ピッチ，ロール角[°]

    if (STEPS == 0) dmSetCamera(x,y,z,yaw,pitch,roll); // カメラの設定

    dmSimQuickStep(); // シミュレーションを１ステップ進める(高速版)

    for (i = 0; i < DOMINO_NUM; i++)
    {
        dmDraw(&domino[i]); //　壁の描画
    }
    STEPS++;
}

void resetSim(int n)
{
    double m = 0.1; // 質量
    double side[3] = {0.2, 0.05, 0.5}; // サイズ
    double R[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0}; // 姿勢
    double p[DOMINO_NUM][3];  // 位置

    int i;

    // シミュレーションの終了
    if (STEPS != 0)
    {
        dmClose();
    }

    dmInit(); // 初期化

    // ドミノの生成
    for (i = 0; i < DOMINO_NUM; i++)
    {
        p[i][2] = 0.25; // ドミノ重心のz座標
        switch (n)
        {
        case 0:
            p[i][0] = 0;             // ドミノ重心のx座標
            p[i][1] = 0.3 * i -3.0;  // ドミノ重心のy座標
            break;
        case 1:
            p[i][0] = 0.3 * i -3.0;
            p[i][1] = p[i][0];
            // z軸周りにπ/4回転させた姿勢　
            dRFromAxisAndAngle(R, 0, 0, 1, - M_PI/4);
            break;
        default:
            printf("Bad number \n");
            break;
        }
        dmCreateBox(&domino[i], p[i], R, m, side, red);
    }
}

void command(int cmd)
{
    switch (cmd)
    {
    case '1':
        resetSim(1);
        break;
    case 'r':
    case 'R':
        resetSim(0);
        break;
    case 'f':
    case 'F':
    {
        double fx = -1.0, fy = 0.0, fz = 0.0;
        dmAddTorque(&domino[0], fx, fy, fz); // x,y,z軸まわりにfx,fy,fzのトルクを付加
        break;
    }
    default:
        printf("Input r, R, f, F key \n");
        break;
    }
}

/*** main関数 ***/
int main()
{
    resetSim(0); // シミュレーションのリセット
    dmLoop(800, 600, simLoop, command);  // ウインドウの幅，高, ループ関数，コマンド関数
    dmClose(); // 終了

    return 0;
}




