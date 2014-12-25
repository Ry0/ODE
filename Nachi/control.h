#ifndef CONTROL_H_
#define CONTROL_H_
#include "area_struct.h"


bool YesorNo();
int input_arg(int argc, char* argv[]);
int CountNumbersOfTextLines(std::string fileName);
void Input_Data(std::string fileName);

void Input_RRT_Data(std::string fileName);            // RRTのすべての枝をインプット
void printRRT();                                      // RRTのすべての枝を描く
void Input_RRTPath_Data(std::string fileName);        // RRTで決定したパスをインプット
void printPath();                                     // RRTで決定したパスを描く
void Input_RRTPath_mod_Data(std::string fileName);    // 間引きで決定したパスをインプット
void printPath_mod();                                 // 間引きで決定したパスを描く

void Pcontrol();
void Vcontrol();

void inverseKinematics();
void PrintAngle(dReal NowJoint[]);
void yugan_a();

void printPosition(std::vector<POINT> &path, int loop, int DrawLength);

void plot(int pause);
#endif