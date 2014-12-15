#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <time.h>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#endif
int WindowWidth  = 352; //ウィンドウの幅
int WindowHeight = 288; //ウィンドウの高さ

static dWorldID world; //ODE世界のID
static dSpaceID space; //衝突空間のID（同じ衝突空間内のオブジェクトのみが衝突する）
static dGeomID  ground;//地面との衝突判定のID
static dJointGroupID contactgroup; //ジョイントグループID
dsFunctions fn;

const int N = 200;//描画する球の数
const dReal   radius = 0.2;//球の半径
const dReal   mass   = 1.0;//球の質量

typedef struct {//球の構造体
  dBodyID body; //動力学計算用のボディ
  dGeomID geom; //衝突計算用のジオメトリ
} MyObject;
MyObject ball[N];//球を描画するための配列

static void nearCallback(void *data, dGeomID o1, dGeomID o2)//2つのオブジェクトに衝突可能性がある場合に呼び出される
{
  const int N = 10;
  dContact contact[N];

  //地面との衝突であるかどうかの判定
  int isGround = ((ground == o1) || (ground == o2));

  //衝突の判定を実際に行う関数　3つ目の引数に衝突の情報を格納する
  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));


  //if (isGround)  {//もし地面との衝突だけを考える場合にはコメントアウトをはずす
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode = dContactBounce;  //衝突モード
      contact[i].surface.mu   = dInfinity;       // 摩擦係数
      contact[i].surface.bounce     = 0.5;       // 反発係数（0 ～ 1）
      contact[i].surface.bounce_vel = 0.0;       // 最低反射速度
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
    }
  //}
}


static void simLoop (int pause)//無限ループ
{
  const dReal *pos,*R;

  dSpaceCollide(space,0,&nearCallback); //衝突判定用の関数
  dWorldStep(world, 0.01);//世界の時間を進める
  dJointGroupEmpty(contactgroup);//ジョイントグループを初期化
  //球の描画を行う
  for(int i=0;i<N;i++){
    dsSetColor(1.0, 0.0, 0.0);            //オブジェクトの色の設定
    pos = dBodyGetPosition(ball[i].body); //オブジェクトの位置情報（posは配列）
    R   = dBodyGetRotation(ball[i].body); //オブジェクトの回転情報（Rは配列）
    dsDrawSphere(pos,R,radius);           //オブジェクトの描画
  }
}

void start() //初期設定
{
  static float xyz[3] = {0.0,-3.0,1.0}; //カメラの位置
  static float hpr[3] = {90.0,0.0,0.0}; //カメラの方向
  //(z軸, y軸, x軸)における回転角度（{0.0,0.0,0.0}でx軸方向を向いている）
  //上の場合は、y軸の方向を向いている
  dsSetViewpoint (xyz,hpr); //カメラの設定
}

void  prepDrawStuff() {   //描画の初期化
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.stop    = NULL;
  fn.path_to_textures = "textures";
}

//いよいよメイン関数
int main (int argc, char *argv[])
{
  srand(time(NULL)); //rand()の種
  dReal x0 = 0.0, y0 = 0.0, z0 = 3.0;
  dMass m1;

  prepDrawStuff();

  dInitODE();
  world = dWorldCreate();      //世界の誕生（IDをworldに代入）
  space = dHashSpaceCreate(0); //衝突空間の生成（IDをspaceに代入）
  contactgroup = dJointGroupCreate(0); //ジョイント空間の生成（IDをcontactgroupに代入）

  dWorldSetGravity(world,0,0,-0.5);//世界の重力加速度を指定

  //地面との衝突判定用ジオメトリの生成（描画なし）
  ground = dCreatePlane(space,0,0,1,0);

  //ボールの生成
  for(int i=0;i<N;i++){
    ball[i].body = dBodyCreate(world);     //動力学計算用ボディのIDを取得
    dMassSetZero(&m1);                     //基本形状構造体の初期化
    dMassSetSphereTotal(&m1, mass ,radius);//基本形状構造体に重心、慣性テンソルのパラメータをセットする（この場合は球）
    dBodySetMass(ball[i].body,&m1);        //生成したオブジェクトと基本形状構造体とリンクさせる
    //生成したオブジェクトの初期位置を設定
    dBodySetPosition(ball[i].body, x0+ double(rand()-rand())/double(RAND_MAX*10)  , y0 + double(rand()-rand())/double(RAND_MAX*10) , z0 + double(i)/2.0);

    ball[i].geom = dCreateSphere(space,radius); //衝突計算用ジオメトリのIDを取得
    dGeomSetBody(ball[i].geom,ball[i].body);    //衝突計算用ジオメトリのIDと動力学計算用ボディのIDをリンク
  }
  dsSimulationLoop (argc,argv,WindowWidth, WindowHeight,&fn); //シミュレーション用の無限ループ

  dWorldDestroy (world);
  dCloseODE();

  return 0;
}