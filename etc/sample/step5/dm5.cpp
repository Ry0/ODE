#include "dm5.h"

dsFunctions fn;

dWorldID world = NULL;                     // 動力学の世界
dSpaceID space = NULL;  // 衝突検出用スペース
dGeomID  ground = NULL; // 地面
dJointGroupID contactgroup = NULL; // コンタクトグループ

/*** 前処理　***/
void start()
{
    static float xyz[3] = {3.0,0.0,1.0};         // 視点の位置
    static float hpr[3] = {-180, 0, 0};          // 視線の方向
    dsSetViewpoint(xyz,hpr);                     // カメラの設定
    dsSetSphereQuality(3);                       // 球を美しく描画
    //dRSetIdentity(R);                            // 回転行列に単位行列を設定
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

/*** コールバック関数 ***/
void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    static const int N = 10; // 接触点数の最大値
    dContact contact[N];     // 接触点

    // 接触している物体のどちらかが地面ならisGroundに非0をセット
    int isGround = ((ground == o1) || (ground == o2));
    isGround = 1;

    // 衝突情報の生成 nは衝突点数
    int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (isGround)
    {
        for (int i = 0; i < n; i++)
        {
            contact[i].surface.mode = dContactBounce; // 接触面の反発性を設定
            contact[i].surface.bounce = 0.1;          // 反発係数(0.0から1.0)
            contact[i].surface.bounce_vel = 0.05;      // 反発に必要な最低速度
            contact[i].surface.mu = 0.1;// dInfinity;        // 摩擦係数

            // 接触ジョイントの生成
            dJointID c = dJointCreateContact(world,contactgroup,
                                             &contact[i]);
            // 接触している２つの剛体を接触ジョイントにより拘束
            dJointAttach(c,dGeomGetBody(contact[i].geom.g1),
                         dGeomGetBody(contact[i].geom.g2));
        }
    }
}

/*** 力を物体の重心に加える　***/
void dmAddForce(dmObject *obj, double fx, double fy, double fz)
{
    dBodyAddForce(obj->body, fx, fy, fz);
}

/*** トルクを物体の重心に加える　***/
void dmAddTorque(dmObject *obj, double fx, double fy, double fz)
{
    dBodyAddTorque(obj->body, fx, fy, fz);
}

/*** シミュレーションの初期化 ***/
void dmInit()
{
    dInitODE();                              // ODEの初期化
    world = dWorldCreate();                  // 世界の創造
    dWorldSetGravity(world,0,0,-9.8);        // 重力設定

    space        = dHashSpaceCreate(0);   // 衝突用空間の創造
    contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
    ground = dCreatePlane(space,0,0,1,0); // 地面（平面ジオメトリ）の生成
}

/*** シミュレーションの終了 ***/
void dmClose()
{
    dWorldDestroy(world);                    // 世界の破壊
    dCloseODE();                             // ODEの終了
}

/*** 視点の設定　***/
void dmSetCamera(double x, double y, double z, double yaw, double pitch, double roll)
{
    float xyz[3], hpr[3];
    xyz[0] = (float) x;
    xyz[1] = (float) y;
    xyz[2] = (float) z;
    hpr[0] = (float) yaw;
    hpr[1] = (float) pitch;
    hpr[2] = (float) roll;
    dsSetViewpoint(xyz, hpr);
}

void dmCreateSphere(dmObject *obj, double p[2], double R[12], double m, double r, double color[3])
{
    obj->body = dBodyCreate(world);          // ボールの生成
    obj->m = m;
    obj->r = r;

    obj->R = R;
    obj->p = p;
    obj->color = color;


    dMass mass;                              // 構造体massの宣言
    dMassSetZero(&mass);                     // 構造体massの初期化
    dMassSetSphereTotal(&mass,obj->m,obj->r);          // 構造体massに質量を設定
    dBodySetMass(obj->body,&mass);               // ボールにmassを設定

    dBodySetPosition(obj->body, obj->p[0], obj->p[1], obj->p[2]);  // ボールの位置(x,y,z)を設定
    dBodySetRotation(obj->body, obj->R);

    obj->geom = dCreateSphere(space,obj->r); // 球ジオメトリの生成
    dGeomSetBody(obj->geom, obj->body);      // ボディとジオメトリの関連付け
}

void dmCreateBox(dmObject *obj, double p[3], double R[12], double m, double side[3],  double color[3])
{
    obj->body = dBodyCreate(world);          // ボールの生成
    obj->m = m;

    obj->R = R;
    obj->p = p;
    obj->side = side;
    obj->color = color;

    dMass mass;                              // 構造体massの宣言
    dMassSetZero(&mass);                     // 構造体massの初期化
    dMassSetBoxTotal(&mass,obj->m,obj->side[0], obj->side[1], obj->side[2]);  // 構造体massに質量を設定
    dBodySetMass(obj->body,&mass);               // ボールにmassを設定

    dBodySetPosition(obj->body, obj->p[0], obj->p[1], obj->p[2]);  // ボールの位置(x,y,z)を設定
    dBodySetRotation(obj->body, obj->R);

    obj->geom = dCreateBox(space,obj->side[0], obj->side[1], obj->side[2]); // 球ジオメトリの生成
    dGeomSetBody(obj->geom, obj->body);      // ボディとジオメトリの関連付け
}

void dmCreateCylinder(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3])
{
    obj->body = dBodyCreate(world);

    obj->m = m;
    obj->r = r;
    obj->l = l;

    obj->R = R;
    obj->p = p;
    obj->color = color;

    dMass mass;                              // 構造体massの宣言
    dMassSetZero(&mass);                     // 構造体massの初期化
    dMassSetBoxTotal(&mass,obj->m, 3, obj->r, obj->l);  // 構造体massに質量を設定
    dBodySetMass(obj->body,&mass);               // ボールにmassを設定

    dBodySetPosition(obj->body, obj->p[0], obj->p[1], obj->p[2]);  // ボールの位置(x,y,z)を設定
    dBodySetRotation(obj->body, obj->R);

    obj->geom = dCreateCylinder(space,obj->r, obj->l); // 円柱ジオメトリの生成
    dGeomSetBody(obj->geom, obj->body);      // ボディとジオメトリの関連付け
}

void dmCreateCapsule(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3])
{
    obj->body = dBodyCreate(world);
    obj->m = m;
    obj->r = r;
    obj->l = l;

    obj->R = R;
    obj->p = p;
    obj->color = color;

    dMass mass;                              // 構造体massの宣言
    dMassSetZero(&mass);                     // 構造体massの初期化
    dMassSetBoxTotal(&mass,obj->m, 3, obj->r, obj->l);  // 構造体massに質量を設定
    dBodySetMass(obj->body,&mass);               // ボールにmassを設定

    dBodySetPosition(obj->body, obj->p[0], obj->p[1], obj->p[2]);  // ボールの位置(x,y,z)を設定
    dBodySetRotation(obj->body, obj->R);

    obj->geom = dCreateCapsule(space,obj->r, obj->l); // 円柱ジオメトリの生成
    dGeomSetBody(obj->geom, obj->body);      // ボディとジオメトリの関連付け
}


void dmSimStep()
{
    dSpaceCollide(space,0,&nearCallback); // 衝突検出関数
    dWorldStep(world,0.01);        // シミュレーションを1ステップ進める（精度が高い）
    dJointGroupEmpty(contactgroup); // ジョイントグループを空にする
}

void dmSimQuickStep()
{
    dSpaceCollide(space,0,&nearCallback); // 衝突検出関数
    dWorldQuickStep(world,0.01);     // シミュレーションを1ステップ進める（高速）
    dJointGroupEmpty(contactgroup); // ジョイントグループを空にする
}

void dmDraw(dmObject *obj) /*** 物体の描画 ***/
{
    if (!obj->geom) return;
    obj->p = (double *) dGeomGetPosition(obj->geom);
    obj->R = (double *) dGeomGetRotation(obj->geom);

    dsSetColor(obj->color[0],obj->color[1],obj->color[2]);  // 色の設定(r,g,b)
    // printf("color=%.1f %.1f %.1f\n",obj->color[0],obj->color[1],obj->color[2]);

    int type = dGeomGetClass(obj->geom);

    switch (type)
    {
    case dBoxClass:
        {
            dVector3 sides;
            dGeomBoxGetLengths(obj->geom,sides);
            dsDrawBox(obj->p,obj->R,sides);
        }
        break;

    case dSphereClass:
        dsDrawSphere(obj->p,obj->R,dGeomSphereGetRadius(obj->geom));
        break;

    case  dCapsuleClass:
        {
            dReal radius,length;
            dGeomCapsuleGetParams(obj->geom,&radius,&length);
            dsDrawCapsule(obj->p,obj->R,length,radius);
        }
        break;

    case dCylinderClass:
        {
            dReal radius,length;
            dGeomCylinderGetParams(obj->geom,&radius,&length);
            dsDrawCylinder(obj->p,obj->R,length,radius);
        }
        break;

    default:
        printf("Bad geometry type \n");
    }
}

void dmLoop(int w, int h, void (*function)(int pause), void (*function2)(int cmd))
{
    setDrawStuff(function,function2);
    dsSimulationLoop(0,0,w,h,&fn);
}
