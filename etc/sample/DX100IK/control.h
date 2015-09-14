#ifndef CONTROL_H_
#define CONTROL_H_
#include "area_struct.h"

void Pcontrol();
void directKinematics();
void inverseKinematics();
void CheckThetaE();
void yugan_a();

void printPosition(std::vector<POINT> &path, int loop, int DrawLength);
void Input_Data(std::string fileName);
int CountNumbersOfTextLines(std::string fileName);

#endif