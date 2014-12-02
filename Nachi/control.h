#ifndef CONTROL_H_
#define CONTROL_H_
#include "area_struct.h"

int CountNumbersOfTextLines(std::string fileName);
void Input_Data(std::string fileName);

void Pcontrol();
void Vcontrol();

void inverseKinematics();
void yugan_a();

void printPosition(std::vector<POINT> &path, int loop);
#endif