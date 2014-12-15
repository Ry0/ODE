/* step3 switch文とキー処理関数　*/
#include "dm3.h"
#include <time.h>
#include <stdlib.h>

double R[12] ={1,0,0,0, 0,1,0,0, 0,0,1,0};  // 回転行列が格納される配列, 位置(x,y,z)[m]
double p[3] = {0.0, 0.0, 0.05};   // 位置(x,y,z)[m]
double sides[3] = {0.1, 0.1, 0.1}; // 直方体のサイズ(x, y, z)[m]

double start_x = 0.0, start_y = 0.05, start_z = 0.0; // 初期位置

void command(int cmd)
{
    float xyz;

    switch (cmd)
    {
    case 'z':
        start_z += 0.1;
        break;
    default:
        printf("Input z key \n");
    }
}


void simLoop(int pause)        /***  シミュレーションループ　***/
{
    int i, j, num = 10; // 直方体の数
    float red = 0.0, green = 0.0, blue = 0.0; // 赤，緑，青成分

    for (i = 0; i < num; i++)
    {
        for (j = 0; j < num; j++)
        {
            red = (float) rand()/RAND_MAX; // 赤成分を乱数で決定
            dsSetColor(red, green, blue); // 色の設定
            p[0] = start_x; // 位置のx成分
            p[1] = i * 0.1 + start_y; // 位置のy成分
            p[2] = j * 0.1 + 0.05 + start_z; // 位置のz成分
            dsDrawBox(p,R,sides); // 直方体の表示
        }
    }
}

int main()         /*** main関数 ***/
{
    srand(time(NULL)); // 乱数の初期化
    dmLoop(800, 600, simLoop, command); // シミュレーションループ ウインドウの幅，高
    return 0;
}
