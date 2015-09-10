#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox      dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#endif
#define ROW 10
#define COL 10

static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
static int flag = 0;
dsFunctions fn;
dMatrix3 R;

const dReal   radius = 0.2;
const dReal   mass   = 1.0;

typedef struct {
  dBodyID body;
  dGeomID geom;
} MyObject;
MyObject ball, pole[2];

typedef struct {
  MyObject ball2[2];
  MyObject ball;
} MyUnion;
MyUnion Union[ROW*COL];

dJointID bodyjoint[2];
typedef struct {
  dJointID bodyjoint[2];
} MyJoint;
MyJoint Joint[ROW*COL];


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

  //srand((unsigned int)time(NULL));
  for (int i = 0; i < 4; i++) {
    theta[i] = M_PI*GetRandom(-2,2,4);
  }
  dRFromEulerAngles(R, theta[1], theta[2], theta[3]);
}


void DropPoint(double ary_x[], double ary_y[], int r, int c, double r_distance, double c_distance){

  for(int i=0; i<r*c; i++){
    for(int j=0; j<c; j++){
      if(i%c==j){
        ary_x[i] = c_distance*j;
      }
    }
  }
  for(int i=0; i<r*c; i++){
    for(int j=0; j<r; j++){
      if(i/r==j){
        ary_y[i] = r_distance*j;
      }
    }
  }
}


void makeball()
{
  dReal x[ROW*COL], y[ROW*COL], z0 = 2.0;
  dMass m1;
  dReal l[2] = {0.2, -0.2};
  // Create a ball

  DropPoint(x, y, ROW, COL, 1, 1);

  for(int i=0; i<ROW*COL; i++){
    Union[i].ball.body = dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetSphereTotal(&m1,mass,radius);
    dBodySetMass(Union[i].ball.body,&m1);
    dBodySetPosition(Union[i].ball.body, x[i], y[i], z0);

    Union[i].ball.geom = dCreateSphere(space,radius);
    dGeomSetBody(Union[i].ball.geom, Union[i].ball.body);

    for(int j=0; j<2; j++){
      Union[i].ball2[j].body = dBodyCreate(world);
      dMassSetZero(&m1);
      dMassSetSphereTotal(&m1,mass,radius);
      dBodySetMass(Union[i].ball2[j].body,&m1);
      dBodySetPosition(Union[i].ball2[j].body, x[i], y[i], z0+l[j]);

      Union[i].ball2[j].geom = dCreateSphere(space,radius);
      dGeomSetBody(Union[i].ball2[j].geom, Union[i].ball2[j].body);
    }

    for(int j=0; j<2; j++){
      Joint[i].bodyjoint[j] = dJointCreateHinge(world,0);
      dJointAttach(Joint[i].bodyjoint[j], Union[i].ball2[j].body, Union[i].ball.body);
      dJointSetHingeAxis(Joint[i].bodyjoint[j],0, 0, 1);
      dJointSetHingeAnchor(Joint[i].bodyjoint[j], x[i], y[i], z0);//ヒンジの中心点(x,y,z)
      dJointSetHingeParam(Joint[i].bodyjoint[j],dParamLoStop, 0);
      dJointSetHingeParam(Joint[i].bodyjoint[j],dParamHiStop, 0);
    }
    Randam();
    dBodySetRotation(Union[i].ball.body, R);
  }
}

void draw()
{

  for(int i=0; i<ROW*COL; i++){
    dsSetColor(1.0, 1.0, 1.0);
    dsDrawSphere(dBodyGetPosition(Union[i].ball.body), dBodyGetRotation(Union[i].ball.body), radius);

    for (int j = 0; j < 2; j++) {
      dsSetColor(0.0, 0.0, 1.0);
      dsDrawSphere(dBodyGetPosition(Union[i].ball2[j].body), dBodyGetRotation(Union[i].ball2[j].body), radius);
    }
  }
}


static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i,n;

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnected(b1, b2))
    return;

  const int N = 4;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
      if (dGeomGetClass(o1) == dSphereClass || dGeomGetClass(o2) == dSphereClass){
        contact[i].surface.mu = 20;
      }else{
        contact[i].surface.mu = 0.5;
      }
      contact[i].surface.slip1 = 0.0;
      contact[i].surface.slip2 = 0.0;
      contact[i].surface.soft_erp = 0.8;
      contact[i].surface.soft_cfm = 0.01;
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach(c,dGeomGetBody(o1),dGeomGetBody(o2));
    }

  }

}



static void simLoop (int pause)
{

  flag = 0;
  dSpaceCollide(space,0,&nearCallback);

  dWorldStep(world,0.01);
  draw();
  dJointGroupEmpty(contactgroup);


}

void start()
{
  static float xyz[3] = {0.0,-3.0,1.0};
  static float hpr[3] = {90.0,0.0,0.0};
  dsSetViewpoint (xyz,hpr);
}

void  prepDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
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

  dWorldSetGravity(world,0,0,-0.5);

  // Create a ground
  ground = dCreatePlane(space,0,0,1,0);

  makeball();

  dsSimulationLoop (argc,argv,640,480,&fn);

  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
