#include "dm6.h"

extern dmObject ball, field, fence[3], bar, block1, block2, block3;
extern int block1_hit, block2_hit, block3_hit;

dWorldID world = NULL;                     // 動力学の世界
dSpaceID space = NULL;  // 衝突検出用スペース
dGeomID ground = NULL; // 地面
dJointGroupID contactgroup = NULL; // コンタクトグループ
dsFunctions fn;

/*** コールバック関数 ***/
void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 10; // 接触点数の最大値
	dContact contact[N];     // 接触点

	// 衝突情報の生成 nは衝突点数
	int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

	for (int i = 0; i < n; i++){
		contact[i].surface.mode = dContactBounce; // 接触面の反発性を設定

		// ボールとバーが接触したら反発係数を1.5に設定
		if(((ball.geom == o1) && (bar.geom == o2)) || ((ball.geom == o2) && (bar.geom == o1))){
			contact[i].surface.bounce = 1.2;          // 反発係数(実際の世界では0.0から1.0)
		}
		// ボールと床が接触したら反発係数を0.5に設定
		else if (((ball.geom == o1) && (field.geom == o2)) || ((ball.geom == o2) && (field.geom == o1))){
			contact[i].surface.bounce = 0.5;
		}
		// ボールとブロック１が接触したら。block1_hitを１に設定
		else if (((ball.geom == o1) && (block1.geom == o2)) || ((ball.geom == o2) && (block1.geom == o1))){
			 block1_hit = 1;
		}
		// ボールとブロック2が接触したら。block2_hitを１に設定
		else if (((ball.geom == o1) && (block2.geom == o2)) || ((ball.geom == o2) && (block2.geom == o1))){
			 block2_hit = 1;
		}
		// ボールとブロック3が接触したら。block2_hitを１に設定
		else if (((ball.geom == o1) && (block3.geom == o2)) || ((ball.geom == o2) && (block3.geom == o1))){
			 block3_hit = 1;
		}
		// その他は反発係数を0.8に設定
		else{
			contact[i].surface.bounce = 0.8;
		}

		//contact[i].surface.bounce_vel = 0.05;            // 反発に必要な最低速度
		contact[i].surface.mu = 0.0;// dInfinity;        // 摩擦係数

		// 接触ジョイントの生成
		dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
		// 接触している２つの剛体を接触ジョイントにより拘束
		dJointAttach(c,dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
	}
}

int dmCollision(dmObject *obj1, dmObject *obj2)
{
	static const int N = 10; // 接触点数の最大値
	dContact contact2[N];     // 接触点

	int n = dCollide(obj1->geom, obj2->geom, N, &contact2[0].geom,sizeof(dContact));
	if (n > 0) printf("dCollide=%d\n",n);
	if (n > 0)  return 1;
	else        return 0;
}


/*** 描画関数の設定 ***/
void setDrawStuff(dmFunctions *dmf)
{
	fn.version = DS_VERSION;    // ドロースタッフのバージョン
	fn.start   = dmf->start;    // 前処理 start関数のポインタ
	fn.step    = dmf->step;     // simLoop関数のポインタ
	fn.command = dmf->command;
	fn.path_to_textures = "./textures"; // テクスチャ
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
	dWorldSetGravity(world, 0.0,0.0,-9.8);        // 重力設定

	space        = dHashSpaceCreate(0);   // 衝突用空間の創造
	contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
	ground = dCreatePlane(space,0,0,1,0); // 地面（平面ジオメトリ）の生成
	dsSetSphereQuality(3);
}

/*** シミュレーションの終了 ***/
void dmClose()
{
	dWorldDestroy(world);                    // 世界の破壊
	dCloseODE();                             // ODEの終了
}

/*** 視点の設定　***/
void dmSetCamera(double x, double y, double z, double roll, double pitch, double yaw)
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

void dmCreateSphere0(dmObject *obj, double p[2], double R[12], double m, double r, double color[3])
{
	obj->body = NULL;
	obj->m = m;
	obj->r = r;
	obj->R = R;
	obj->p = p;
	obj->color = color;

	obj->geom = dCreateSphere(space,obj->r); // 球ジオメトリの生成
	dGeomSetPosition(obj->geom, obj->p[0], obj->p[1], obj->p[2]);  // ボールの位置(x,y,z)を設定
	dGeomSetRotation(obj->geom, obj->R);
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

void dmCreateBox0(dmObject *obj, double p[3], double R[12], double m, double side[3], double color[3])
{
	obj->body = NULL;
	obj->side = side;
	obj->color = color;
	obj->p = p;
	obj->R = R;

	obj->geom = dCreateBox(space,obj->side[0], obj->side[1], obj->side[2]); // ボックスジオメトリの生成
	dGeomSetPosition(obj->geom, obj->p[0], obj->p[1], obj->p[2]);  // ボールの位置(x,y,z)を設定
	dGeomSetRotation(obj->geom, obj->R);
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

void dmCreateCylinder0(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3])
{
	obj->body = NULL;
	obj->r = r;
	obj->l = l;
	obj->color = color;
	obj->p = p;
	obj->R = R;

	obj->geom = dCreateCylinder(space,obj->r, obj->l); // 円柱ジオメトリの生成
	dGeomSetPosition(obj->geom, obj->p[0], obj->p[1], obj->p[2]);
	dGeomSetRotation(obj->geom, obj->R);
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

void dmCreateCapsule0(dmObject *obj, double p[3], double R[12], double m, double r, double l, double color[3])
{
	obj->body = NULL;
	obj->r = r;
	obj->l = l;
	obj->color = color;
	obj->p = p;
	obj->R = R;

	obj->geom = dCreateCapsule(space,obj->r, obj->l); // 円柱ジオメトリの生成
	dGeomSetPosition(obj->geom, obj->p[0], obj->p[1], obj->p[2]);  // ボールの位置(x,y,z)を設定
	dGeomSetRotation(obj->geom, obj->R);
}


void dmWorldStep()
{
	dSpaceCollide(space,0,&nearCallback); // 衝突検出関数
	dWorldStep(world,0.01);        // シミュレーションを1ステップ進める（精度が高い）
	dJointGroupEmpty(contactgroup); // ジョイントグループを空にする
}

void dmWorldQuickStep()
{
	dSpaceCollide(space,0,&nearCallback); // 衝突検出関数
	dWorldQuickStep(world,0.01);     // シミュレーションを1ステップ進める（高速）
	dJointGroupEmpty(contactgroup); // ジョイントグループを空にする
}

// ジオメトリのみの物体を破壊する関数。例：ブロック崩しのブロック等
void dmDestroyBox0(dmObject *obj)
{
	if (obj->geom != NULL)  // ジオメトリが存在していれば
	{
		dGeomDestroy(obj->geom); // ジオメトリを破壊
		obj->geom = NULL;            // ポインタにNULLを設定
	}
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

void dmLoop(int w, int h, dmFunctions *dmf)
{
	setDrawStuff(dmf);
	dsSimulationLoop(0,0,w,h,&fn);
}
