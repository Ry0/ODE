#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <sys/time.h>

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#endif

static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;

static int judge = 0;
static int cnt = 0;

dsFunctions fn;
dMatrix3 R;

typedef struct {
  dBodyID body;
  dGeomID geom;
} MyObject;

MyObject dice, one, two[2], three[3], four[4], five[5], six[6];
dJointID bodyjoint1, bodyjoint2[2], bodyjoint3[3], bodyjoint4[4], bodyjoint5[5], bodyjoint6[6];


double GetRandom(double min,double max, int digit){
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


void makedice()
{
  dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;

  dReal leng = 0.4;

  dMatrix3 R_25,R_34;

  dMass mass;
  dReal m = 1.0;
  dReal l = 0.00002;
  dReal r = 0.04;

  dReal  one_mass  = 0.00006;// 質量
  //dReal  one_mass  = 3;// 質量(イカサマ用)
  dReal  one_r = 0.08;
  dReal  one_start_x = 0.0;// 重心 x
  dReal  one_start_y = 0.0;// 重心 y
  dReal  one_start_z = 2.20001;// 重心 z
  dReal  one_hinge_z = 2.2;

  dReal  two_mass  = 0.00003;// 質量
  dReal  two_start_x[2] = {0.1, -0.1};// 重心 x
  dReal  two_start_y = 0.20001;// 重心 y
  dReal  two_start_z[2] = {1.9, 2.1};// 重心 z
  dReal  two_hinge_x[2] = {0.1, -0.1};
  dReal  two_hinge_y = 0.2;
  dReal  two_hinge_z[2] = {1.9, 2.1};

  dReal  three_mass  = 0.00002;// 質量
  dReal  three_start_x = -0.20001;// 重心 y
  dReal  three_start_y[3] = {0.1, 0.0, -0.1};// 重心 y
  dReal  three_start_z[3] = {2.1, 2.0, 1.9};// 重心 z
  dReal  three_hinge_x = -0.2;
  dReal  three_hinge_y[3] = {0.1, 0.0, -0.1};;
  dReal  three_hinge_z[3] = {2.1, 2.0, 1.9};

  dReal  four_mass  = 0.000015;// 質量
  dReal  four_start_x = 0.20001;// 重心 y
  dReal  four_start_y[4] = {0.1, 0.1, -0.1, -0.1};// 重心 y
  dReal  four_start_z[4] = {1.9, 2.1, 1.9, 2.1};// 重心 z
  dReal  four_hinge_x = 0.2;
  dReal  four_hinge_y[4] = {0.1, 0.1, -0.1, -0.1};
  dReal  four_hinge_z[4] = {1.9, 2.1, 1.9, 2.1};

  dReal  five_mass  = 0.000012;// 質量
  dReal  five_start_x[5] = {0.1, 0.1, 0.0, -0.1, -0.1};// 重心 x
  dReal  five_start_y = -0.20001;// 重心 y
  dReal  five_start_z[5] = {1.9, 2.1, 2.0, 1.9, 2.1};// 重心 z
  dReal  five_hinge_x[5] = {0.1, 0.1, 0.0, -0.1, -0.1};
  dReal  five_hinge_y = -0.2;
  dReal  five_hinge_z[5] = {1.9, 2.1, 2.0, 1.9, 2.1};

  dReal  six_mass  = 0.00001;// 質量
  dReal  six_start_x[6] = {0.0, 0.0, 0.1, 0.1, -0.1, -0.1};// 重心 x
  dReal  six_start_y[6] = {0.1, -0.1, 0.1, -0.1, 0.1, -0.1};// 重心 y
  dReal  six_start_z = 1.79999;// 重心 z
  dReal  six_hinge_x[6] = {0.0, 0.0, 0.1, 0.1, -0.1, -0.1};
  dReal  six_hinge_y[6] = {0.1, -0.1, 0.1, -0.1, 0.1, -0.1};
  dReal  six_hinge_z = 1.8;


/*-------------------------------------------------回転行列計算----------------------------------------------------------*/
  dRFromAxisAndAngle(R_25, 1, 0, 0, M_PI/2.0);
  dRFromAxisAndAngle(R_34, 0, 1, 0, M_PI/2.0);

/*-------------------------------------------------サイコロ作成----------------------------------------------------------*/
  dice.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,m,leng,leng,leng);
  dBodySetMass(dice.body,&mass);
  dBodySetPosition(dice.body, x0, y0, z0);

  dice.geom = dCreateBox(space,leng,leng,leng);
  dGeomSetBody(dice.geom,dice.body);

/*----------------------------------------------------1の目--------------------------------------------------------------*/
  one.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass,one_mass, 3, one_r, l);
  dBodySetMass(one.body, &mass);
  dBodySetPosition(one.body, one_start_x, one_start_y, one_start_z);

  one.geom = dCreateCapsule(space, one_r, l);
  dGeomSetBody(one.geom, one.body);

/*----------------------------------------------------2の目--------------------------------------------------------------*/
  for (int i = 0; i < 2; i++) {
    two[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,two_mass, 3, r, l);;
    dBodySetMass(two[i].body, &mass);
    dBodySetPosition(two[i].body, two_start_x[i], two_start_y, two_start_z[i]);

    two[i].geom = dCreateCapsule(space, r, l);
    dGeomSetBody(two[i].geom, two[i].body);
    dBodySetRotation(two[i].body, R_25);
  }

/*----------------------------------------------------3の目--------------------------------------------------------------*/
  for (int i = 0; i < 3; i++) {
    three[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,three_mass, 3, r, l);;
    dBodySetMass(three[i].body, &mass);
    dBodySetPosition(three[i].body, three_start_x, three_start_y[i], three_start_z[i]);

    three[i].geom = dCreateCapsule(space, r, l);
    dGeomSetBody(three[i].geom, four[i].body);
    dBodySetRotation(three[i].body, R_34);
  }

/*----------------------------------------------------4の目--------------------------------------------------------------*/
  for (int i = 0; i < 4; i++) {
    four[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,four_mass, 3, r, l);;
    dBodySetMass(four[i].body, &mass);
    dBodySetPosition(four[i].body, four_start_x, four_start_y[i], four_start_z[i]);

    four[i].geom = dCreateCapsule(space, r, l);
    dGeomSetBody(four[i].geom, four[i].body);
    dBodySetRotation(four[i].body, R_34);
  }

/*----------------------------------------------------5の目--------------------------------------------------------------*/
  for (int i = 0; i < 5; i++) {
    five[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,five_mass, 3, r, l);;
    dBodySetMass(five[i].body, &mass);
    dBodySetPosition(five[i].body, five_start_x[i], five_start_y, five_start_z[i]);

    five[i].geom = dCreateCapsule(space, r, l);
    dGeomSetBody(five[i].geom, five[i].body);
    dBodySetRotation(five[i].body, R_25);
  }

/*----------------------------------------------------6の目--------------------------------------------------------------*/
  for (int i = 0; i < 6; i++) {
    six[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,six_mass, 3, r, l);;
    dBodySetMass(six[i].body, &mass);
    dBodySetPosition(six[i].body, six_start_x[i], six_start_y[i], six_start_z);

    six[i].geom = dCreateCapsule(space, r, l);
    dGeomSetBody(six[i].geom, six[i].body);
  }

/*---------------------------------------------------ジョイント----------------------------------------------------------*/
  bodyjoint1 = dJointCreateHinge(world,0);
  dJointAttach(bodyjoint1, one.body, dice.body);
  dJointSetHingeAxis(bodyjoint1,0, 0, 1);
  dJointSetHingeAnchor(bodyjoint1, 0, 0, one_hinge_z);//ヒンジの中心点(x,y,z)
  dJointSetHingeParam(bodyjoint1,dParamLoStop, 0);
  dJointSetHingeParam(bodyjoint1,dParamHiStop, 0);


  for (int i = 0; i < 2; i++) {
    bodyjoint2[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint2[i], two[i].body, dice.body);
    dJointSetHingeAxis(bodyjoint2[i], 0, 1, 0);
    dJointSetHingeAnchor(bodyjoint2[i], two_hinge_x[i], two_hinge_y, two_hinge_z[i]);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint2[i], dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint2[i], dParamHiStop, 0);
  }

  for (int i = 0; i < 3; i++) {
    bodyjoint3[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint3[i], three[i].body, dice.body);
    dJointSetHingeAxis(bodyjoint3[i], 1, 0, 0);
    dJointSetHingeAnchor(bodyjoint3[i], three_hinge_x, three_hinge_y[i], three_hinge_z[i]);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint3[i], dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint3[i], dParamHiStop, 0);
  }

  for (int i = 0; i < 4; i++) {
    bodyjoint4[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint4[i], four[i].body, dice.body);
    dJointSetHingeAxis(bodyjoint4[i], 1, 0, 0);
    dJointSetHingeAnchor(bodyjoint4[i], four_hinge_x, four_hinge_y[i], four_hinge_z[i]);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint4[i], dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint4[i], dParamHiStop, 0);
  }

  for (int i = 0; i < 5; i++) {
    bodyjoint5[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint5[i], five[i].body, dice.body);
    dJointSetHingeAxis(bodyjoint5[i], 0, 1, 0);
    dJointSetHingeAnchor(bodyjoint5[i], five_hinge_x[i], five_hinge_y, five_hinge_z[i]);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint5[i], dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint5[i], dParamHiStop, 0);
  }

  for (int i = 0; i < 6; i++) {
    bodyjoint6[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint6[i], six[i].body, dice.body);
    dJointSetHingeAxis(bodyjoint6[i], 0, 0, 1);
    dJointSetHingeAnchor(bodyjoint6[i], six_hinge_x[i], six_hinge_y[i], six_hinge_z);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint6[i], dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint6[i], dParamHiStop, 0);
  }

/*------------------------------------------乱数によって決まった物体の姿勢----------------------------------------------*/
  Randam();
  dBodySetRotation(dice.body,R);
}


void draw()
{
  dReal length1[] = {0.4, 0.4, 0.4};

  dsSetColor(1.0, 1.0, 1.0);
  dsDrawBox(dBodyGetPosition(dice.body), dBodyGetRotation(dice.body), length1);

  dsSetColor(1.0, 0.0, 0.0);
  dsDrawCylinder(dBodyGetPosition(one.body), dBodyGetRotation(one.body), 0.00002, 0.08);

  dsSetColor(0.0, 0.0, 0.0);
  for (int i = 0; i < 2; i++) {
    dsDrawCylinder(dBodyGetPosition(two[i].body), dBodyGetRotation(two[i].body), 0.00002, 0.04);
  }

  for (int i = 0; i < 3; i++) {
    dsDrawCylinder(dBodyGetPosition(three[i].body), dBodyGetRotation(three[i].body), 0.00002, 0.04);
  }

  for (int i = 0; i < 4; i++) {
    dsDrawCylinder(dBodyGetPosition(four[i].body), dBodyGetRotation(four[i].body), 0.00002, 0.04);
  }

  for (int i = 0; i < 5; i++) {
    dsDrawCylinder(dBodyGetPosition(five[i].body), dBodyGetRotation(five[i].body), 0.00002, 0.04);
  }

  for (int i = 0; i < 6; i++) {
    dsDrawCylinder(dBodyGetPosition(six[i].body), dBodyGetRotation(six[i].body), 0.00002, 0.04);
  }
}


static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 1000;
  dContact contact[N];

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

  if((o1==dice.geom && o2 == ground)||
     (o2==dice.geom && o1 == ground)){
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode = dContactBounce;
      contact[i].surface.mu   = dInfinity;
      contact[i].surface.bounce     = 0.5;
      contact[i].surface.bounce_vel = 0.4;
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
    }
  }
}


int detame()
{
  const dReal *linear_vel;
  const dReal *pos1, *pos2, *pos3, *pos4, *pos5, *pos6;
  double pos[6];
  double max = 0.0;
  int max_num = 0.0;

  linear_vel  = dBodyGetLinearVel(dice.body);

  if(cnt>=300 && linear_vel[0] <= 0.00000001 && linear_vel[1] <= 0.00000001 && linear_vel[2] <= 0.00000001 ){

    pos1 = dBodyGetPosition(one.body);
    pos2 = dBodyGetPosition(two[0].body);
    pos3 = dBodyGetPosition(three[0].body);
    pos4 = dBodyGetPosition(four[0].body);
    pos5 = dBodyGetPosition(five[0].body);
    pos6 = dBodyGetPosition(six[0].body);

    pos[0] = pos1[2];
    pos[1] = pos2[2];
    pos[2] = pos3[2];
    pos[3] = pos4[2];
    pos[4] = pos5[2];
    pos[5] = pos6[2];

    max = pos[0];
    max_num = 0;
    for(int i = 1; i < 6; i++){
      if(pos[i]>=max){
        max = pos[i];
        max_num = i;
      }
    }
    if(judge == 0){
      printf("出た目は %d ！！\n", max_num+1);
      judge = 1;
    }
  }else{
    max_num = -1;
  }

  return max_num+1;
}


void resetSimulation()
{
  makedice();
  cnt = 0;
  judge = 0;
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
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,0.01);
  draw();
  detame();
  dJointGroupEmpty(contactgroup);
  cnt++;
}


void start()
{
  static float xyz[3] = {0.0,0.0,4.0};
  static float hpr[3] = {90.0,-90.0,0.0};
  dsSetViewpoint (xyz,hpr);
}


void  prepDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = &command;;
  fn.stop    = NULL;
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

  dsSimulationLoop (argc,argv,400,400,&fn);

  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
