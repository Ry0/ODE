#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#define ROW 5
#define COL 5

#ifdef dDOUBLE
#define dsDrawBox			dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#endif

static dWorldID world;
static dSpaceID space;
static dGeomID	ground;
static dJointGroupID contactgroup;
//static int flag = 0;
static int max_print_flag = 0;
static int judge[ROW*COL] = {0};
static int check = 0;
static int cnt = 0;

dsFunctions fn;
dMatrix3 R;

typedef struct {
	dBodyID body;
	dGeomID geom;
} MyObject;


typedef struct {
	MyObject box;
	MyObject one;
	MyObject two[2];
	MyObject three[3];
	MyObject four[4];
	MyObject five[5];
	MyObject six[6];
} MyUnion;
MyUnion Dice[ROW*COL];


typedef struct {
	dJointID bodyjoint1;
	dJointID bodyjoint2[2];
	dJointID bodyjoint3[3];
	dJointID bodyjoint4[4];
	dJointID bodyjoint5[5];
	dJointID bodyjoint6[6];
} MyJoint;
MyJoint Joint[ROW*COL];


double GetRandom(double min,double max, int digit)
{
	double ten,R;
	static int srand_flag = 0;

	if (srand_flag == 0) {
		srand((unsigned int)time(NULL));
		srand_flag = 1;
	}

	ten = pow(10,digit-1);
	R = min*ten + (int)(rand()*((max-min)*ten+1.0)/(1.0+RAND_MAX));
	return R/ten;
}


void Randam()
{
	dReal theta[4];

	for (int i = 0; i < 4; i++) {
		theta[i] = M_PI*GetRandom(-2,2,4);
	}
	dRFromEulerAngles(R, theta[1], theta[2], theta[3]);
}


void DropPoint(double ary_x[], double ary_y[], int r, int c, double r_distance, double c_distance)
{
	for(int i=0; i<r*c; i++){
		for(int j=0; j<c; j++){
			if(i%c==j){
				ary_x[i] = c_distance*j-(double)((c_distance*(c-1))/2.0);
			}
		}
	}
	for(int i=0; i<r*c; i++){
		for(int j=0; j<r; j++){
			if(i/r==j){
				ary_y[i] = r_distance*j-(double)((r_distance*(r-1))/2.0);
			}
		}
	}
}


void makedice()
{
	dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;

	dReal leng = 0.4;

	dMatrix3 R_25,R_34;

	dMass mass;
	dReal m = 1.0;
	dReal l = 0.00002;
	dReal r = 0.04;

	dReal	one_mass	= 0.00006;// 質量
	//dReal	one_mass	= 3;// 質量(イカサマ用)
	dReal	one_r = 0.08;
	dReal	one_start_x = 0.0;// 重心 x
	dReal	one_start_y = 0.0;// 重心 y
	dReal	one_start_z = 2.20001;// 重心 z
	dReal	one_hinge_z = 2.2;

	dReal	two_mass	= 0.00003;// 質量
	dReal	two_start_x[2] = {0.1, -0.1};// 重心 x
	dReal	two_start_y = 0.20001;// 重心 y
	dReal	two_start_z[2] = {1.9, 2.1};// 重心 z
	dReal	two_hinge_x[2] = {0.1, -0.1};
	dReal	two_hinge_y = 0.2;
	dReal	two_hinge_z[2] = {1.9, 2.1};

	dReal	three_mass	= 0.00002;// 質量
	dReal	three_start_x = -0.20001;// 重心 y
	dReal	three_start_y[3] = {0.1, 0.0, -0.1};// 重心 y
	dReal	three_start_z[3] = {2.1, 2.0, 1.9};// 重心 z
	dReal	three_hinge_x = -0.2;
	dReal	three_hinge_y[3] = {0.1, 0.0, -0.1};;
	dReal	three_hinge_z[3] = {2.1, 2.0, 1.9};

	dReal	four_mass	= 0.000015;// 質量
	dReal	four_start_x = 0.20001;// 重心 y
	dReal	four_start_y[4] = {0.1, 0.1, -0.1, -0.1};// 重心 y
	dReal	four_start_z[4] = {1.9, 2.1, 1.9, 2.1};// 重心 z
	dReal	four_hinge_x = 0.2;
	dReal	four_hinge_y[4] = {0.1, 0.1, -0.1, -0.1};
	dReal	four_hinge_z[4] = {1.9, 2.1, 1.9, 2.1};

	dReal	five_mass	= 0.000012;// 質量
	dReal	five_start_x[5] = {0.1, 0.1, 0.0, -0.1, -0.1};// 重心 x
	dReal	five_start_y = -0.20001;// 重心 y
	dReal	five_start_z[5] = {1.9, 2.1, 2.0, 1.9, 2.1};// 重心 z
	dReal	five_hinge_x[5] = {0.1, 0.1, 0.0, -0.1, -0.1};
	dReal	five_hinge_y = -0.2;
	dReal	five_hinge_z[5] = {1.9, 2.1, 2.0, 1.9, 2.1};

	dReal	six_mass	= 0.00001;// 質量
	dReal	six_start_x[6] = {0.0, 0.0, 0.1, 0.1, -0.1, -0.1};// 重心 x
	dReal	six_start_y[6] = {0.1, -0.1, 0.1, -0.1, 0.1, -0.1};// 重心 y
	dReal	six_start_z = 1.79999;// 重心 z
	dReal	six_hinge_x[6] = {0.0, 0.0, 0.1, 0.1, -0.1, -0.1};
	dReal	six_hinge_y[6] = {0.1, -0.1, 0.1, -0.1, 0.1, -0.1};
	dReal	six_hinge_z = 1.8;

	dReal x[ROW*COL], y[ROW*COL];

	DropPoint(x, y, ROW, COL, 1.1, 1.1);

	for(int i=0; i<ROW*COL; i++){
/*-------------------------------------------------回転行列計算----------------------------------------------------------*/
		dRFromAxisAndAngle(R_25, 1, 0, 0, M_PI/2.0);
		dRFromAxisAndAngle(R_34, 0, 1, 0, M_PI/2.0);
/*-------------------------------------------------サイコロ作成----------------------------------------------------------*/
		Dice[i].box.body = dBodyCreate(world);
		dMassSetZero(&mass);
		dMassSetBoxTotal(&mass, m, leng, leng, leng);
		dBodySetMass(Dice[i].box.body,&mass);
		dBodySetPosition(Dice[i].box.body, x0+x[i], y0+y[i], z0);

		Dice[i].box.geom = dCreateBox(space, leng, leng, leng);
		dGeomSetBody(Dice[i].box.geom, Dice[i].box.body);

/*----------------------------------------------------1の目--------------------------------------------------------------*/
		Dice[i].one.body = dBodyCreate(world);
		dMassSetZero(&mass);
		dMassSetCapsuleTotal(&mass, one_mass, 3, one_r, l);
		dBodySetMass(Dice[i].one.body, &mass);
		dBodySetPosition(Dice[i].one.body, one_start_x+x[i], one_start_y+y[i], one_start_z);

		Dice[i].one.geom = dCreateCapsule(space, one_r, l);
		dGeomSetBody(Dice[i].one.geom, Dice[i].one.body);

/*----------------------------------------------------2の目--------------------------------------------------------------*/
		for (int j = 0; j < 2; j++) {
			Dice[i].two[j].body	= dBodyCreate(world);
			dMassSetZero(&mass);
			dMassSetCapsuleTotal(&mass,two_mass, 3, r, l);;
			dBodySetMass(Dice[i].two[j].body, &mass);
			dBodySetPosition(Dice[i].two[j].body, two_start_x[j]+x[i], two_start_y+y[i], two_start_z[j]);

			Dice[i].two[j].geom = dCreateCapsule(space, r, l);
			dGeomSetBody(Dice[i].two[j].geom, Dice[i].two[j].body);
			dBodySetRotation(Dice[i].two[j].body, R_25);
		}

/*----------------------------------------------------3の目--------------------------------------------------------------*/
		for (int j = 0; j < 3; j++) {
			Dice[i].three[j].body	= dBodyCreate(world);
			dMassSetZero(&mass);
			dMassSetCapsuleTotal(&mass,three_mass, 3, r, l);;
			dBodySetMass(Dice[i].three[j].body, &mass);
			dBodySetPosition(Dice[i].three[j].body, three_start_x+x[i], three_start_y[j]+y[i], three_start_z[j]);

			Dice[i].three[j].geom = dCreateCapsule(space, r, l);
			dGeomSetBody(Dice[i].three[j].geom, Dice[i].four[j].body);
			dBodySetRotation(Dice[i].three[j].body, R_34);
		}

/*----------------------------------------------------4の目--------------------------------------------------------------*/
		for (int j = 0; j < 4; j++) {
			Dice[i].four[j].body	= dBodyCreate(world);
			dMassSetZero(&mass);
			dMassSetCapsuleTotal(&mass,four_mass, 3, r, l);;
			dBodySetMass(Dice[i].four[j].body, &mass);
			dBodySetPosition(Dice[i].four[j].body, four_start_x+x[i], four_start_y[j]+y[i], four_start_z[j]);

			Dice[i].four[j].geom = dCreateCapsule(space, r, l);
			dGeomSetBody(Dice[i].four[j].geom, Dice[i].four[j].body);
			dBodySetRotation(Dice[i].four[j].body, R_34);
		}

/*----------------------------------------------------5の目--------------------------------------------------------------*/
		for (int j = 0; j < 5; j++) {
			Dice[i].five[j].body	= dBodyCreate(world);
			dMassSetZero(&mass);
			dMassSetCapsuleTotal(&mass,five_mass, 3, r, l);;
			dBodySetMass(Dice[i].five[j].body, &mass);
			dBodySetPosition(Dice[i].five[j].body, five_start_x[j]+x[i], five_start_y+y[i], five_start_z[j]);

			Dice[i].five[j].geom = dCreateCapsule(space, r, l);
			dGeomSetBody(Dice[i].five[j].geom, Dice[i].five[j].body);
			dBodySetRotation(Dice[i].five[j].body, R_25);
		}

/*----------------------------------------------------6の目--------------------------------------------------------------*/
		for (int j = 0; j < 6; j++) {
			Dice[i].six[j].body	= dBodyCreate(world);
			dMassSetZero(&mass);
			dMassSetCapsuleTotal(&mass,six_mass, 3, r, l);;
			dBodySetMass(Dice[i].six[j].body, &mass);
			dBodySetPosition(Dice[i].six[j].body, six_start_x[j]+x[i], six_start_y[j]+y[i], six_start_z);

			Dice[i].six[j].geom = dCreateCapsule(space, r, l);
			dGeomSetBody(Dice[i].six[j].geom, Dice[i].six[j].body);
		}

/*---------------------------------------------------ジョイント----------------------------------------------------------*/
		Joint[i].bodyjoint1 = dJointCreateHinge(world,0);
		dJointAttach(Joint[i].bodyjoint1, Dice[i].one.body, Dice[i].box.body);
		dJointSetHingeAxis(Joint[i].bodyjoint1, 0, 0, 1);
		dJointSetHingeAnchor(Joint[i].bodyjoint1, x[i], y[i], one_hinge_z);//ヒンジの中心点(x,y,z)
		dJointSetHingeParam(Joint[i].bodyjoint1, dParamLoStop, 0);
		dJointSetHingeParam(Joint[i].bodyjoint1, dParamHiStop, 0);


		for (int j = 0; j < 2; j++) {
			Joint[i].bodyjoint2[j] = dJointCreateHinge(world,0);
			dJointAttach(Joint[i].bodyjoint2[j], Dice[i].two[j].body, Dice[i].box.body);
			dJointSetHingeAxis(Joint[i].bodyjoint2[j], 0, 1, 0);
			dJointSetHingeAnchor(Joint[i].bodyjoint2[j], two_hinge_x[j]+x[i], two_hinge_y+y[i], two_hinge_z[j]);//ヒンジの中心点(x,y,z)
			dJointSetHingeParam(Joint[i].bodyjoint2[j], dParamLoStop, 0);
			dJointSetHingeParam(Joint[i].bodyjoint2[j], dParamHiStop, 0);
		}

		for (int j = 0; j < 3; j++) {
			Joint[i].bodyjoint3[j] = dJointCreateHinge(world,0);
			dJointAttach(Joint[i].bodyjoint3[j], Dice[i].three[j].body, Dice[i].box.body);
			dJointSetHingeAxis(Joint[i].bodyjoint3[j], 1, 0, 0);
			dJointSetHingeAnchor(Joint[i].bodyjoint3[j], three_hinge_x+x[i], three_hinge_y[j]+y[i], three_hinge_z[j]);//ヒンジの中心点(x,y,z)
			dJointSetHingeParam(Joint[i].bodyjoint3[j], dParamLoStop, 0);
			dJointSetHingeParam(Joint[i].bodyjoint3[j], dParamHiStop, 0);
		}

		for (int j = 0; j < 4; j++) {
			Joint[i].bodyjoint4[j] = dJointCreateHinge(world,0);
			dJointAttach(Joint[i].bodyjoint4[j], Dice[i].four[j].body, Dice[i].box.body);
			dJointSetHingeAxis(Joint[i].bodyjoint4[j], 1, 0, 0);
			dJointSetHingeAnchor(Joint[i].bodyjoint4[j], four_hinge_x+x[i], four_hinge_y[j]+y[i], four_hinge_z[j]);//ヒンジの中心点(x,y,z)
			dJointSetHingeParam(Joint[i].bodyjoint4[j], dParamLoStop, 0);
			dJointSetHingeParam(Joint[i].bodyjoint4[j], dParamHiStop, 0);
		}

		for (int j = 0; j < 5; j++) {
			Joint[i].bodyjoint5[j] = dJointCreateHinge(world,0);
			dJointAttach(Joint[i].bodyjoint5[j], Dice[i].five[j].body, Dice[i].box.body);
			dJointSetHingeAxis(Joint[i].bodyjoint5[j], 0, 1, 0);
			dJointSetHingeAnchor(Joint[i].bodyjoint5[j], five_hinge_x[j]+x[i], five_hinge_y+y[i], five_hinge_z[j]);//ヒンジの中心点(x,y,z)
			dJointSetHingeParam(Joint[i].bodyjoint5[j], dParamLoStop, 0);
			dJointSetHingeParam(Joint[i].bodyjoint5[j], dParamHiStop, 0);
		}

		for (int j = 0; j < 6; j++) {
			Joint[i].bodyjoint6[j] = dJointCreateHinge(world,0);
			dJointAttach(Joint[i].bodyjoint6[j], Dice[i].six[j].body, Dice[i].box.body);
			dJointSetHingeAxis(Joint[i].bodyjoint6[j], 0, 0, 1);
			dJointSetHingeAnchor(Joint[i].bodyjoint6[j], six_hinge_x[j]+x[i], six_hinge_y[j]+y[i], six_hinge_z);//ヒンジの中心点(x,y,z)
			dJointSetHingeParam(Joint[i].bodyjoint6[j], dParamLoStop, 0);
			dJointSetHingeParam(Joint[i].bodyjoint6[j], dParamHiStop, 0);
		}

/*------------------------------------------乱数によって決まった物体の姿勢----------------------------------------------*/
		Randam();
		dBodySetRotation(Dice[i].box.body, R);

	}
}


void draw()
{
	dReal length1[] = {0.4, 0.4, 0.4};

	for(int i=0; i<ROW*COL; i++){
		dsSetColor(1.0, 1.0, 1.0);
		dsDrawBox(dBodyGetPosition(Dice[i].box.body), dBodyGetRotation(Dice[i].box.body), length1);

		dsSetColor(1.0, 0.0, 0.0);
		dsDrawCylinder(dBodyGetPosition(Dice[i].one.body), dBodyGetRotation(Dice[i].one.body), 0.00002, 0.08);

		dsSetColor(0.0, 0.0, 0.0);
		for (int j = 0; j < 2; j++) {
			dsDrawCylinder(dBodyGetPosition(Dice[i].two[j].body), dBodyGetRotation(Dice[i].two[j].body), 0.00002, 0.04);
		}

		for (int j = 0; j < 3; j++) {
			dsDrawCylinder(dBodyGetPosition(Dice[i].three[j].body), dBodyGetRotation(Dice[i].three[j].body), 0.00002, 0.04);
		}

		for (int j = 0; j < 4; j++) {
			dsDrawCylinder(dBodyGetPosition(Dice[i].four[j].body), dBodyGetRotation(Dice[i].four[j].body), 0.00002, 0.04);
		}

		for (int j = 0; j < 5; j++) {
			dsDrawCylinder(dBodyGetPosition(Dice[i].five[j].body), dBodyGetRotation(Dice[i].five[j].body), 0.00002, 0.04);
		}

		for (int j = 0; j < 6; j++) {
			dsDrawCylinder(dBodyGetPosition(Dice[i].six[j].body), dBodyGetRotation(Dice[i].six[j].body), 0.00002, 0.04);
		}
	}
}


int Collision_if(dGeomID o1, dGeomID o2)
{
	int a = 0;

	for(int i=0; i<ROW*COL; i++){
		if(((o1==Dice[i].box.geom)&&(o2==ground))||
			((o2==Dice[i].box.geom)&&(o1==ground))){
			a++;
		}
	}
	return a;
}


int Collision_if2(dGeomID o1, dGeomID o2)
{
	int a = 0;

	for(int i=0; i<ROW*COL; i++){
		for(int j=0; j<ROW*COL; j++){
			if(((o1==Dice[i].box.geom)&&(o2==Dice[j].box.geom))||
				((o2==Dice[i].box.geom)&&(o1==Dice[j].box.geom))){
				a++;
			}
		}
	}
	return a;
}


static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	const int N = 1000;
	dContact contact[N];

	int n =	dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

	if(Collision_if(o1,o2)>0 || Collision_if2(o1,o2)>0){
		// if (n >= 1) flag = 1;
		// else				flag = 0;
		for (int i = 0; i < n; i++) {
			contact[i].surface.mode = dContactBounce;
			contact[i].surface.mu	 = dInfinity;
			contact[i].surface.bounce		 = 0.5; // (0.0~1.0) restitution parameter
			contact[i].surface.bounce_vel = 0.4; // minimum incoming velocity for bounce
			dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
			dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
		}
	}
}


int detame(int dice_num){
	const dReal *linear_vel;
	const dReal *pos1;
	const dReal *pos2;
	const dReal *pos3;
	const dReal *pos4;
	const dReal *pos5;
	const dReal *pos6;
	double pos[6];
	double max = 0.0;
	int max_num = 0.0;

	linear_vel	= dBodyGetLinearVel(Dice[dice_num].box.body);

	if(cnt>=300 && linear_vel[0] <= 0.00000001 && linear_vel[1] <= 0.00000001 && linear_vel[2] <= 0.00000001 ){

		pos1 = dBodyGetPosition(Dice[dice_num].one.body);
		pos2 = dBodyGetPosition(Dice[dice_num].two[0].body);
		pos3 = dBodyGetPosition(Dice[dice_num].three[0].body);
		pos4 = dBodyGetPosition(Dice[dice_num].four[0].body);
		pos5 = dBodyGetPosition(Dice[dice_num].five[0].body);
		pos6 = dBodyGetPosition(Dice[dice_num].six[0].body);

		pos[0] = pos1[2];
		pos[1] = pos2[2];
		pos[2] = pos3[2];
		pos[3] = pos4[2];
		pos[4] = pos5[2];
		pos[5] = pos6[2];

		max = pos[0];
		max_num = 0;
		for(int j = 1; j < 6; j++){
			if(pos[j]>=max){
				max = pos[j];
				max_num = j;
			}
		}
		if(judge[dice_num] == 0){
			printf("%2d番目のサイコロの出た目は %d ！！\n", dice_num+1, max_num+1);
			judge[dice_num] = 1;
		}else{
			max_num = -1;
		}
	}else{
		max_num = -1;
	}
	return max_num+1;
}


int detame_sum()
{
	static int sum[ROW*COL] = {0};
	static int temp[ROW*COL] = {0};
	int dice_sum = 0;

	for(int i=0; i<ROW*COL; i++){
		temp[i] = detame(i);
	}

	for(int i=0; i<ROW*COL; i++){
		if(temp[i]>0){
			sum[i] = temp[i];
			check++;
		}
	}

	if(check == ROW*COL){
		dice_sum = 0;
		for(int i=0; i<ROW*COL; i++){
			dice_sum = dice_sum + sum[i];
		}
		if(max_print_flag==0){
			printf("出た目の合計 = %d\n", dice_sum);
			max_print_flag = 1;
		}
	}else{
		dice_sum = 0;
	}

	return dice_sum;
}


void resetSimulation()
{
	makedice();
	cnt = 0;
	check = 0;
	max_print_flag = 0;
	for(int i=0; i<ROW*COL; i++){
		judge[i] = 0;
	}
}


static void command (int cmd)
{
	switch (cmd) {
		case 'r': case 'R':
			printf("サイコロリセット！\n");
			resetSimulation();
		break;
			default:
			printf("入力したキーは「%c」です。「r」を押してサイコロリセットです。\n",(char)cmd);
	}
}


static void simLoop (int pause)
{
	//flag = 0;
	dSpaceCollide(space,0,&nearCallback);
	dWorldStep(world,0.01);
	draw();
	detame_sum();

	dJointGroupEmpty(contactgroup);
	cnt++;
}


void start()
{
	static float xyz[3] = {0.0,0.0,7.0};
	static float hpr[3] = {90.0,-90.0,0.0};
	dsSetViewpoint (xyz,hpr);
}


void	prepDrawStuff() {
	fn.version = DS_VERSION;
	fn.start	 = &start;
	fn.step		= &simLoop;
	fn.command = &command;;
	fn.stop		= NULL;
	fn.path_to_textures = "./textures";
}


int main (int argc, char *argv[])
{

	prepDrawStuff();

	dInitODE();
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(world,0,0,-9.8);
	ground = dCreatePlane(space,0,0,1,0);

	makedice();

	dsSimulationLoop (argc,argv,640,480,&fn);

	dWorldDestroy (world);
	dCloseODE();

	return 0;
}
