#include "area_struct.h"
#include "obstacle_struct.h"

/*---------------------------------------------------------------------- ↓グローバル変数定義ここから↓ -------------------------------------------------------------------------------*/
extern dWorldID      world;                   // 動力学計算用のワールド
extern dSpaceID      space;                   // 衝突検出用のスペース
extern dGeomID       ground;                  // 地面のジオメトリID番号
extern dJointGroupID contactgroup;            // 接触点グループ
extern MyObject boxparts;
extern dJointID boxjoint;
extern dsFunctions   fn;                      // ドロースタッフの描画関数
extern dReal StartP[3],GoalP[3];

extern int numObstacle;
extern double *xMin, *xMax;
extern double *yMin, *yMax;
extern double *zMin, *zMax;
/*---------------------------------------------------------------------- ↑グローバル変数定義ここまで↑ -------------------------------------------------------------------------------*/

void initObstacleFromFile(std::string fileName)
{
  if (xMin != NULL)
    delete [] xMin;
  if (xMax != NULL)
    delete [] xMax;
  if (yMin != NULL)
    delete [] yMin;
  if (yMax != NULL)
    delete [] yMax;
  if (zMin != NULL)
    delete [] zMin;
  if (zMax != NULL)
    delete [] zMax;

  std::ifstream input(fileName.c_str());

  double xLeft, xRight, yBottom, yTop, zBottom, zTop;
  input >> xLeft >> xRight >> yBottom >> yTop >> zBottom >> zTop >> numObstacle;

  xMin = new double[numObstacle];
  xMax = new double[numObstacle];
  yMin = new double[numObstacle];
  yMax = new double[numObstacle];
  zMin = new double[numObstacle];
  zMax = new double[numObstacle];

  for (int i = 0; i < numObstacle; ++i){
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i] >> zMin[i] >> zMax[i];
  }

  input.close();
  cout << "障害物リスト" << endl;
  for (int i = 0; i < numObstacle; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf] z[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i], zMin[i], zMax[i]);
  }

}



void drawBox()
{
  double R;
  double G = 191.0/255.0;
  double B = 255.0/255.0;
  dReal length[3] = {0.0};
  dReal GravityCenter[3] = {0.0};// 重心 xyz
  dMatrix3 tmpR;

  for (int i = 0; i < numObstacle; ++i){
    R = i * 1.0/(double)numObstacle;
    #ifdef Skeleton
    dsSetColor(R,G,B);
    #else
    dsSetColorAlpha(R, G, B, 0.3);
    #endif

    length[0] = xMax[i] - xMin[i];
    length[1] = yMax[i] - yMin[i];
    length[2] = zMax[i] - zMin[i];
    GravityCenter[0] = (xMax[i] + xMin[i])/2.0;
    GravityCenter[1] = (yMax[i] + yMin[i])/2.0;
    GravityCenter[2] = (zMax[i] + zMin[i])/2.0;
    dRSetIdentity(tmpR);
    dsDrawBox(GravityCenter, tmpR, length);
  }

}



void drawStartandGoal()
{
  dMatrix3 tmpR1,tmpR2;

  double R = 255.0/255.0;
  double G = 20.0/255.0;
  double B = 147.0/255.0;

  dsSetColor(R, G, B);

  dRSetIdentity(tmpR1);
  dsDrawSphere(StartP, tmpR1, 0.02);
  dRSetIdentity(tmpR2);
  dsDrawSphere(GoalP, tmpR2, 0.02);
}