#ifndef CONTROL_H_
#define CONTROL_H_
#include "area_struct.h"


bool YesorNo();
int input_arg(int argc, char* argv[]);
int CountNumbersOfTextLines(std::string fileName);
void Input_Data(std::string fileName);

void Pcontrol();
void Vcontrol();

void inverseKinematics();
void PrintAngle(dReal NowJoint[]);
void yugan_a();

void printPosition(std::vector<POINT> &path, int loop);

void plot(int pause);
#endif