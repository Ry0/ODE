#include "area_struct.h"
#include "obstacle_struct.h"

extern dWorldID      world;                   // 動力学計算用のワールド
extern dSpaceID      space;                   // 衝突検出用のスペース
extern dGeomID       ground;                  // 地面のジオメトリID番号
extern dJointGroupID contactgroup;            // 接触点グループ
extern MyObject boxparts;
extern dJointID boxjoint;
extern dsFunctions   fn;                      // ドロースタッフの描画関数


void makeBox()
{
  dMass mass;                                    // 質量パラメータ

  dReal  box_mass  = 0.02;// 質量
  dReal  box_x_length = 0.1;
  dReal  box_y_length = 0.1;
  dReal  box_z_length = 0.9;

  dReal  box_start_x = 0.35;// 重心 y
  dReal  box_start_y = 0.05;// 重心 y
  dReal  box_start_z = 0.45;// 重心 z

  boxparts.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, box_mass, box_x_length, box_y_length, box_z_length);
  dBodySetMass(boxparts.body, &mass);

  boxparts.geom = dCreateBox(space, box_x_length, box_y_length, box_z_length);
  dGeomSetBody(boxparts.geom, boxparts.body);
  dBodySetPosition(boxparts.body, box_start_x, box_start_y, box_start_z);

  //パーツの合体
  boxjoint = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(boxjoint, boxparts.body, 0);
  dJointSetFixed(boxjoint);

}


void drawBox()
{
  dReal box_length[] = {0.1, 0.1, 0.9};

  double R = 0.0/255.0;
  double G = 191.0/255.0;
  double B = 255.0/255.0;
  //dsSetColor(R,G,B);
  dsSetColorAlpha(R, G, B, 0.25);

  dsDrawBox(dBodyGetPosition(boxparts.body), dBodyGetRotation(boxparts.body), box_length);

  dReal tmpP1[3],tmpP2[3];
  dMatrix3 tmpR1,tmpR2;

  tmpP1[0] = 0.35;
  tmpP1[1] = 0.35;
  tmpP1[2] = 0.40;

  tmpP2[0] = 0.2;
  tmpP2[1] = -0.35;
  tmpP2[2] = 0.10;

  double R2 = 255.0/255.0;
  double G2 = 20.0/255.0;
  double B2 = 147.0/255.0;

  dsSetColor(R2,G2,B2);

  dRSetIdentity(tmpR1);
  dsDrawSphere(tmpP1, tmpR1, 0.02);
  dRSetIdentity(tmpR2);
  dsDrawSphere(tmpP2, tmpR2, 0.02);

}
