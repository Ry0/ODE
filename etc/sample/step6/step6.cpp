/* step6 ドミノ倒し 2012-01-19 　*/
#include "dm6.h"
#define COLUMN 10 // 列
#define ROW     3 // 行

static int STEPS = 0;                                        // シミュレーションのステップ数
double red[3]      = {1.3, 0.0, 0.0};                        // 赤色
double yellow[3]  = {1.3, 1.3, 0.0};                         // 黄色
double green[3]  = {0.0, 0.8, 0.0};                          // 緑色
double blue[3]    = {0.0, 0.0, 1.3};                         // 青
int block1_hit = 0, block2_hit = 0, block3_hit = 0;          // ブロックにボールが衝突すれば１、しないと０

dmObject block1, block2, block3, ball, field, fence[3], bar; // ドミノ, フィールド, 柵
dmObject block[ROW][COLUMN];                                 // ブロックを多く表示する場合に使う。未実装。
dmFunctions dmf;                                             // 描画関数の構造体

void resetSim(int n)
{
	double m = 0.1;                                                   // 質量
	double side[3] = {0.2, 0.05, 0.5};                                // サイズ
	double R[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};                       // 姿勢
	double p[3];                                                      // 位置
	double ball_pos[3]= {-0.5, 0, 0.65}, ball_r = 0.03, ball_m = 0.1; // ボールの位置、半径、質量
	double bar_pos[3]= {-0.09, 0, 0.6}, bar_side[3] = {0.01, 0.1, 0.1}, bar_m = 0.1;                      // ボールの位置、半径、質量
	double block1_pos[3], block2_pos[3], block3_pos[3], block_side[3] = {0.08, 0.08, 0.1}, block_m = 0.1; // ブロックの位置、半径、質量
	double *block_color[3] = {red, blue, yellow};
	double field_pos[3]= {-0.5, 0, 0.5}, field_side[3] = {1, 1, 0.01}, field_m;                           // 柵の位置、サイズ, 質量
	double fence_pos[3][3] = {{-1.0, 0, 0.6}, {-0.5, -0.5, 0.6},{-0.5, 0.5, 0.6}};
	double fence_side[3][3] = {{0.05, 1.05, 0.1}, {1.0, 0.05, 0.1}, {1.0, 0.05, 0.1}};
	double fence_m[3] = {0.1, 0.1, 0.1};
	int i, j;


	// シミュレーションの終了
	if (STEPS != 0)
	{
		dmClose();
	}

	// ブロックのあたり判定変数を０に初期化
	block1_hit = 0, block2_hit = 0, block3_hit = 0;

	dmInit(); // 初期化

	// ボールの生成
	dmCreateSphere(&ball, ball_pos, R, ball_m, ball_r, red);
	dBodyAddForce(ball.body, 0, 0.1, 0);

	// バーの生成
	dmCreateBox0(&bar, bar_pos, R, bar_m, bar_side, yellow);

	// フィールドの生成
	dmCreateBox0(&field, field_pos, R, field_m, field_side, green);
	for (i =0; i < 3; i++)
	{
		dmCreateBox0(&fence[i], fence_pos[i], R, fence_m[i], fence_side[i], green);
	}

	// ブロックの生成
	double p0 = -0.5, p1 = -0.5;

	// ブロック１の位置設定
	block1_pos[0] = -0.7;
	block1_pos[1] = -0.4;
	block1_pos[2] =   0.6;

	// ブロック２の位置設定
	block2_pos[0] = -0.7 - 0.09;
	block2_pos[1] = -0.4 + 0.09* 3;
	block2_pos[2] =   0.6;

	// ブロック３の位置設定
	block3_pos[0] = -0.7 - 0.09* 2;
	block3_pos[1] = -0.4 + 0.09* 6;
	block3_pos[2] =   0.6;

	// ブロックの生成
	dmCreateBox0(&block1, block1_pos, R, block_m, block_side, block_color[0]);
	dmCreateBox0(&block2, block2_pos, R, block_m, block_side, block_color[1]);
	dmCreateBox0(&block3, block3_pos, R, block_m, block_side, block_color[2]);
}

/***  シミュレーションループ　***/
void simLoop(int pause)
{
	int i, j;

	dmAddForce(&ball, 0.0001, 0, 0);
	dmWorldQuickStep(); // シミュレーションを１ステップ進める(高速版)

	dmDraw(&ball);
	dmDraw(&bar);
	dmDraw(&field);

	for (i =0; i < 3; i++)
	{
		dmDraw(&fence[i]);
	}

	// ブロックがボールに衝突していない場合はブロックを表示し
	// ブロックがボールに衝突した場合はブロックを破壊する。
	if (block1_hit==0) dmDraw(&block1);
	else                  dmDestroyBox0(&block1);

	if (block2_hit==0) dmDraw(&block2);
	else                  dmDestroyBox0(&block2);

	if (block3_hit==0) dmDraw(&block3);
	else                  dmDestroyBox0(&block3);

	STEPS++;
}



/*** キ―入力関数 ***/
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
		double pos[3], *p;
		p = pos;
		//double fx = 0.0, fy = 1.0, fz = 0.0;
		//dmAddForce(&ball, fx, fy, fz); // x,y,z軸まわりにfx,fy,fzのトルクを付加
		p = (double *) dGeomGetPosition(bar.geom);
		dGeomSetPosition(bar.geom, p[0], p[1]-=0.05, p[2]);
		break;
	}
	case 'j':
	case 'J':
	{
		double pos[3], *p;
		p = pos;
		p = (double *) dGeomGetPosition(bar.geom);
		dGeomSetPosition(bar.geom, p[0], p[1]+=0.05, p[2]);
		break;
	}
	default:
		printf("Input r, R, f, F key \n");
		break;
	}
}

/*** カメラの位置と姿勢設定 ***/
void setCamera()
{
	float x =  -0.5, y = 0.0, z = 1.5;    // カメラの位置
	float roll = 0, pitch = -90, yaw = -180; // カメラの方向[°]
	dmSetCamera(x,y,z,roll,pitch,yaw);  // カメラの設定
}

/*** 描画用構造体の設定　***/
void setDraw()
{
	dmf.start   = &setCamera;
	dmf.step    = &simLoop;
	dmf.command = &command;
}

/*** main関数 ***/
int main()
{
	resetSim(0); // シミュレーションのリセット
	setDraw();   // 描画関数の設定

	dmLoop(800, 600, &dmf);  // ウインドウの幅，高, ループ関数，コマンド関数
	dmClose(); // 終了

	return 0;
}




