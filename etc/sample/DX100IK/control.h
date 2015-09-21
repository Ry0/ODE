#ifndef CONTROL_H_
#define CONTROL_H_
#include "area_struct.h"

void Pcontrol();
void directKinematics();
bool inverseKinematics(double Theta[]);
bool CheckThetaE();
void CheckTheta();
double AdjustTheta();
void OptimizationThetaE(int i);
void yugan_a();

void printPosition(std::vector<POINT> &path, int loop, int DrawLength);
void Input_Data(std::string fileName);
void EInput_Data(std::string fileName);
int CountNumbersOfTextLines(std::string fileName);

#endif